/*
  This file contains definitions for functions to compute transforms from
  image feature correspondences
  
  Copyright (C) 2006-2012  Rob Hess <rob@iqengines.com>

  @version 1.1.2-20100521
*/

#include "xform.h"
#include "imgfeatures.h"
#include "utils.h"

#include <cxcore.h>

#include <stdlib.h>
#include <time.h>

/************************* Local Function Prototypes *************************/

static inline struct feature* get_match( struct feature*, int );
static int get_matched_features( struct feature*, int, int, struct feature*** );
static int calc_min_inliers( int, int, double, double );
static inline double log_factorial( int );
static struct feature** draw_ransac_sample( struct feature**, int, int );
static void extract_corresp_pts( struct feature**, int, int, CvPoint2D64f**,
			  CvPoint2D64f** );
static int find_consensus( struct feature**, int, int, CvMat*, ransac_err_fn,
		    double, struct feature*** );
static inline void release_mem( CvPoint2D64f*, CvPoint2D64f*,
				struct feature** );

/********************** Functions prototyped in model.h **********************/


/*
  Calculates a best-fit image transform from image feature correspondences
  using RANSAC.
  
  For more information refer to:
  
  Fischler, M. A. and Bolles, R. C.  Random sample consensus: a paradigm for
  model fitting with applications to image analysis and automated cartography.
  <EM>Communications of the ACM, 24</EM>, 6 (1981), pp. 381--395.
  
  @param features an array of features; only features with a non-NULL match
    of type mtype are used in homography computation
  @param n number of features in feat
  @param mtype determines which of each feature's match fields to use
    for model computation; should be one of FEATURE_FWD_MATCH,
    FEATURE_BCK_MATCH, or FEATURE_MDL_MATCH; if this is FEATURE_MDL_MATCH,
    correspondences are assumed to be between a feature's img_pt field
    and its match's mdl_pt field, otherwise correspondences are assumed to
    be between the the feature's img_pt field and its match's img_pt field
  @param xform_fn pointer to the function used to compute the desired
    transformation from feature correspondences
  @param m minimum number of correspondences necessary to instantiate the
    model computed by xform_fn
  @param p_badxform desired probability that the final transformation
    returned by RANSAC is corrupted by outliers (i.e. the probability that
    no samples of all inliers were drawn)
  @param err_fn pointer to the function used to compute a measure of error
    between putative correspondences and a computed model
  @param err_tol correspondences within this distance of a computed model are
    considered as inliers
  @param inliers if not NULL, output as an array of pointers to the final
    set of inliers
  @param n_in if not NULL and \a inliers is not NULL, output as the final
    number of inliers
  
  @return Returns a transformation matrix computed using RANSAC or NULL
    on error or if an acceptable transform could not be computed.
*/
CvMat* ransac_xform(struct feature* features, int n, int mtype,
	ransac_xform_fn xform_fn, int m, double p_badxform,
	ransac_err_fn err_fn, double err_tol,
	struct feature*** inliers, int* n_in)
{
	//matched：所有具有mtype类型匹配点的特征点的数组，也就是样本集  
	//sample：单个样本，即4个特征点的数组  
	//consensus：当前一致集；  
	//consensus_max：当前最大一致集(即当前的最好结果的一致集)  
	struct feature** matched, ** sample, ** consensus, ** consensus_max = NULL;
	struct ransac_data* rdata;//每个特征点的feature_data域的ransac数据指针  
	CvPoint2D64f* pts, *mpts;//每个样本对应的两个坐标数组：特征点坐标数组pts和匹配点坐标数组mpts  
	CvMat* M = NULL;//当前变换矩阵  
					//p：当前计算出的模型的错误概率，当p小于p_badxform时迭代停止  
					//in_frac：内点数目占样本总数目的百分比  
	double p, in_frac = RANSAC_INLIER_FRAC_EST;
	//nm：输入的特征点数组中具有mtype类型匹配点的特征点个数  
	//in：当前一致集中元素个数  
	//in_min：一致集中元素个数允许的最小值，保证RANSAC最终计算出的转换矩阵错误的概率小于p_badxform所需的最小内点数目  
	//in_max：当前最优一致集(最大一致集)中元素的个数  
	//k：迭代次数，与计算当前模型的错误概率有关  
	int i, nm, in, in_min, in_max = 0, k = 0;

	//找到特征点数组features中所有具有mtype类型匹配点的特征点，放到matched数组(样本集)中，返回值nm是matched数组的元素个数  
	nm = get_matched_features(features, n, mtype, &matched);
	//若找到的具有匹配点的特征点个数小于计算变换矩阵需要的最小特征点对个数，出错  
	if (nm < m)
	{   //出错处理，特征点对个数不足  
		fprintf(stderr, "Warning: not enough matches to compute xform, %s" \
			" line %d\n", __FILE__, __LINE__);
		goto end;
	}

	/* initialize random number generator */
	srand(time(NULL));//初始化随机数生成器  

					  //计算保证RANSAC最终计算出的转换矩阵错误的概率小于p_badxform所需的最小内点数目  
	in_min = calc_min_inliers(nm, m, RANSAC_PROB_BAD_SUPP, p_badxform);
	//当前计算出的模型的错误概率,内点所占比例in_frac越大，错误概率越小；迭代次数k越大，错误概率越小  
	p = pow(1.0 - pow(in_frac, m), k);
	i = 0;

	//当前错误概率大于输入的允许错误概率p_badxform，继续迭代  
	while (p > p_badxform)
	{
		//从样本集matched中随机抽选一个RANSAC样本(即一个4个特征点的数组)，放到样本变量sample中  
		sample = draw_ransac_sample(matched, nm, m);
		//从样本中获取特征点和其对应匹配点的二维坐标，分别放到输出参数pts和mpts中  
		extract_corresp_pts(sample, m, mtype, &pts, &mpts);
		//调用参数中传入的函数xform_fn，计算将m个点的数组pts变换为mpts的矩阵，返回变换矩阵给M  
		M = xform_fn(pts, mpts, m);//一般传入lsq_homog()函数  
		if (!M)//出错判断  
			goto iteration_end;
		//给定特征点集，变换矩阵，误差函数，计算出当前一致集consensus，返回一致集中元素个数给in  
		in = find_consensus(matched, nm, mtype, M, err_fn, err_tol, &consensus);

		//若当前一致集大于历史最优一致集，即当前一致集为最优，则更新最优一致集consensus_max  
		if (in > in_max)
		{
			if (consensus_max)//若之前有最优值，释放其空间  
				free(consensus_max);
			consensus_max = consensus;//更新最优一致集  
			in_max = in;//更新最优一致集中元素个数  
			in_frac = (double)in_max / nm;//最优一致集中元素个数占样本总个数的百分比  
		}
		else//若当前一致集小于历史最优一致集，释放当前一致集  
			free(consensus);
		cvReleaseMat(&M);

	iteration_end:
		release_mem(pts, mpts, sample);
		p = pow(1.0 - pow(in_frac, m), ++k);
	}

	//根据最优一致集计算最终的变换矩阵  
	/* calculate final transform based on best consensus set */
	//若最优一致集中元素个数大于最低标准，即符合要求  
	if (in_max >= in_min)
	{
		//从最优一致集中获取特征点和其对应匹配点的二维坐标，分别放到输出参数pts和mpts中  
		extract_corresp_pts(consensus_max, in_max, mtype, &pts, &mpts);
		//调用参数中传入的函数xform_fn，计算将in_max个点的数组pts变换为mpts的矩阵，返回变换矩阵给M  
		M = xform_fn(pts, mpts, in_max);
		/***********下面会再进行一次迭代**********/
		//根据变换矩阵M从样本集matched中计算出一致集consensus，返回一致集中元素个数给in  
		in = find_consensus(matched, nm, mtype, M, err_fn, err_tol, &consensus);
		cvReleaseMat(&M);
		release_mem(pts, mpts, consensus_max);
		//从一致集中获取特征点和其对应匹配点的二维坐标，分别放到输出参数pts和mpts中  
		extract_corresp_pts(consensus, in, mtype, &pts, &mpts);
		//调用参数中传入的函数xform_fn，计算将in个点的数组pts变换为mpts的矩阵，返回变换矩阵给M  
		M = xform_fn(pts, mpts, in);
		if (inliers)
		{
			*inliers = consensus;//将最优一致集赋值给输出参数：inliers，即内点集合  
			consensus = NULL;
		}
		if (n_in)
			*n_in = in;//将最优一致集中元素个数赋值给输出参数：n_in，即内点个数  
		release_mem(pts, mpts, consensus);
	}
	else if (consensus_max)
	{   //没有计算出符合要求的一致集  
		if (inliers)
			*inliers = NULL;
		if (n_in)
			*n_in = 0;
		free(consensus_max);
	}

	//RANSAC算法结束：恢复特征点中被更改的数据域feature_data，并返回变换矩阵M  
end:
	for (i = 0; i < nm; i++)
	{
		//利用宏feat_ransac_data来提取matched[i]中的feature_data成员并转换为ransac_data格式的指针  
		rdata = feat_ransac_data(matched[i]);
		//恢复feature_data成员的以前的值  
		matched[i]->feature_data = rdata->orig_feat_data;
		free(rdata);//释放内存  
	}
	free(matched);

	return M;//返回求出的变换矩阵M  
}



/*
  Calculates a planar homography from point correspondeces using the direct
  linear transform.  Intended for use as a ransac_xform_fn.
  
  @param pts array of points
  @param mpts array of corresponding points; each pts[i], i=0..n-1,
    corresponds to mpts[i]
  @param n number of points in both pts and mpts; must be at least 4
  
  @return Returns the 3x3 planar homography matrix that transforms points
    in pts to their corresponding points in mpts or NULL if fewer than 4
    correspondences were provided
*/
CvMat* dlt_homog( CvPoint2D64f* pts, CvPoint2D64f* mpts, int n )
{
  CvMat* H, * A, * VT, * D, h, v9;
  double _h[9];
  int i;

  if( n < 4 )
    return NULL;

  /* set up matrices so we can unstack homography into h; Ah = 0 */
  A = cvCreateMat( 2*n, 9, CV_64FC1 );
  cvZero( A );
  for( i = 0; i < n; i++ )
    {
      cvmSet( A, 2*i, 3, -pts[i].x );
      cvmSet( A, 2*i, 4, -pts[i].y );
      cvmSet( A, 2*i, 5, -1.0  );
      cvmSet( A, 2*i, 6, mpts[i].y * pts[i].x );
      cvmSet( A, 2*i, 7, mpts[i].y * pts[i].y );
      cvmSet( A, 2*i, 8, mpts[i].y );
      cvmSet( A, 2*i+1, 0, pts[i].x );
      cvmSet( A, 2*i+1, 1, pts[i].y );
      cvmSet( A, 2*i+1, 2, 1.0  );
      cvmSet( A, 2*i+1, 6, -mpts[i].x * pts[i].x );
      cvmSet( A, 2*i+1, 7, -mpts[i].x * pts[i].y );
      cvmSet( A, 2*i+1, 8, -mpts[i].x );
    }
  D = cvCreateMat( 9, 9, CV_64FC1 );
  VT = cvCreateMat( 9, 9, CV_64FC1 );
  cvSVD( A, D, NULL, VT, CV_SVD_MODIFY_A + CV_SVD_V_T );
  v9 = cvMat( 1, 9, CV_64FC1, NULL );
  cvGetRow( VT, &v9, 8 );
  h = cvMat( 1, 9, CV_64FC1, _h );
  cvCopy( &v9, &h, NULL );
  h = cvMat( 3, 3, CV_64FC1, _h );
  H = cvCreateMat( 3, 3, CV_64FC1 );
  cvConvert( &h, H );

  cvReleaseMat( &A );
  cvReleaseMat( &D );
  cvReleaseMat( &VT );
  return H;
}



/*
  Calculates a least-squares planar homography from point correspondeces.
  
  @param pts array of points
  @param mpts array of corresponding points; each pts[i], i=1..n, corresponds
    to mpts[i]
  @param n number of points in both pts and mpts; must be at least 4
  
  @return Returns the 3 x 3 least-squares planar homography matrix that
    transforms points in pts to their corresponding points in mpts or NULL if
    fewer than 4 correspondences were provided
*/
CvMat* lsq_homog( CvPoint2D64f* pts, CvPoint2D64f* mpts, int n )
{
  CvMat* H, * A, * B, X;
  double x[9];
  int i;

  if( n < 4 )
    {
      fprintf( stderr, "Warning: too few points in lsq_homog(), %s line %d\n",
	       __FILE__, __LINE__ );
      return NULL;
    }

  /* set up matrices so we can unstack homography into X; AX = B */
  A = cvCreateMat( 2*n, 8, CV_64FC1 );
  B = cvCreateMat( 2*n, 1, CV_64FC1 );
  X = cvMat( 8, 1, CV_64FC1, x );
  H = cvCreateMat(3, 3, CV_64FC1);
  cvZero( A );
  for( i = 0; i < n; i++ )
    {
      cvmSet( A, i, 0, pts[i].x );
      cvmSet( A, i+n, 3, pts[i].x );
      cvmSet( A, i, 1, pts[i].y );
      cvmSet( A, i+n, 4, pts[i].y );
      cvmSet( A, i, 2, 1.0 );
      cvmSet( A, i+n, 5, 1.0 );
      cvmSet( A, i, 6, -pts[i].x * mpts[i].x );
      cvmSet( A, i, 7, -pts[i].y * mpts[i].x );
      cvmSet( A, i+n, 6, -pts[i].x * mpts[i].y );
      cvmSet( A, i+n, 7, -pts[i].y * mpts[i].y );
      cvmSet( B, i, 0, mpts[i].x );
      cvmSet( B, i+n, 0, mpts[i].y );
    }
  cvSolve( A, B, &X, CV_SVD );
  x[8] = 1.0;
  X = cvMat( 3, 3, CV_64FC1, x );
  cvConvert( &X, H );

  cvReleaseMat( &A );
  cvReleaseMat( &B );
  return H;
}



/*
  Calculates the transfer error between a point and its correspondence for
  a given homography, i.e. for a point x, it's correspondence x', and
  homography H, computes d(x', Hx)^2.
  
  @param pt a point
  @param mpt pt's correspondence
  @param H a homography matrix
  
  @return Returns the transfer error between pt and mpt given H
*/
double homog_xfer_err( CvPoint2D64f pt, CvPoint2D64f mpt, CvMat* H )
{
  CvPoint2D64f xpt = persp_xform_pt( pt, H );
  
  return sqrt( dist_sq_2D( xpt, mpt ) );
}



/*
  Performs a perspective transformation on a single point.  That is, for a
  point (x, y) and a 3 x 3 matrix T this function returns the point
  (u, v), where
  
  [x' y' w']^T = T * [x y 1]^T,
  
  and
  
  (u, v) = (x'/w', y'/w').

  Note that affine transforms are a subset of perspective transforms.
  
  @param pt a 2D point
  @param T a perspective transformation matrix
  
  @return Returns the point (u, v) as above.
*/
CvPoint2D64f persp_xform_pt( CvPoint2D64f pt, CvMat* T )
{
  CvMat XY, UV;
  double xy[3] = { pt.x, pt.y, 1.0 }, uv[3] = { 0 };
  CvPoint2D64f rslt;

  cvInitMatHeader( &XY, 3, 1, CV_64FC1, xy, CV_AUTOSTEP );
  cvInitMatHeader( &UV, 3, 1, CV_64FC1, uv, CV_AUTOSTEP );
  cvMatMul( T, &XY, &UV );
  rslt = cvPoint2D64f( uv[0] / uv[2], uv[1] / uv[2] );

  return rslt;
}


/************************ Local funciton definitions *************************/

/*
  Returns a feature's match according to a specified match type

  @param feat feature
  @param mtype match type, one of FEATURE_FWD_MATCH, FEATURE_BCK_MATCH, or
    FEATURE_MDL_MATCH

  @return Returns feat's match corresponding to mtype or NULL for bad mtype
*/
static inline struct feature* get_match( struct feature* feat, int mtype )
{
  if( mtype == FEATURE_MDL_MATCH )
    return feat->mdl_match;
  if( mtype == FEATURE_BCK_MATCH )
    return feat->bck_match;
  if( mtype == FEATURE_FWD_MATCH )
    return feat->fwd_match;
  return NULL;
}



/*
  Finds all features with a match of a specified type and stores pointers
  to them in an array.  Additionally initializes each matched feature's
  feature_data field with a ransac_data structure.

  @param features array of features
  @param n number of features in features
  @param mtype match type, one of FEATURE_{FWD,BCK,MDL}_MATCH
  @param matched output as an array of pointers to features with a match of
    the specified type

  @return Returns the number of features output in matched.
*/
static int get_matched_features( struct feature* features, int n, int mtype,
				 struct feature*** matched )
{
  struct feature** _matched;
  struct ransac_data* rdata;
  int i, m = 0;

  _matched = calloc( n, sizeof( struct feature* ) );
  for( i = 0; i < n; i++ )
    if( get_match( features + i, mtype ) )
      {
	rdata = malloc( sizeof( struct ransac_data ) );
	memset( rdata, 0, sizeof( struct ransac_data ) );
	rdata->orig_feat_data = features[i].feature_data;
	_matched[m] = features + i;
	_matched[m]->feature_data = rdata;
	m++;
      }
  *matched = _matched;
  return m;
}



/*
  Calculates the minimum number of inliers as a function of the number of
  putative correspondences.  Based on equation (7) in
  
  Chum, O. and Matas, J.  Matching with PROSAC -- Progressive Sample Consensus.
  In <EM>Conference on Computer Vision and Pattern Recognition (CVPR)</EM>,
  (2005), pp. 220--226.

  @param n number of putative correspondences
  @param m min number of correspondences to compute the model in question
  @param p_badsupp prob. that a bad model is supported by a correspondence
  @param p_badxform desired prob. that the final transformation returned is bad
  
  @return Returns the minimum number of inliers required to guarantee, based
    on p_badsupp, that the probability that the final transformation returned
    by RANSAC is less than p_badxform
*/
static int calc_min_inliers( int n, int m, double p_badsupp, double p_badxform )
{
  double pi, sum;
  int i, j;

  for( j = m+1; j <= n; j++ )
    {
      sum = 0;
      for( i = j; i <= n; i++ )
	{
	  pi = (i-m) * log( p_badsupp ) + (n-i+m) * log( 1.0 - p_badsupp ) +
	    log_factorial( n - m ) - log_factorial( i - m ) -
	    log_factorial( n - i );
	  /*
	   * Last three terms above are equivalent to log( n-m choose i-m )
	   */
	  sum += exp( pi );
	}
      if( sum < p_badxform )
	break;
    }
  return j;
}



/*
  Calculates the natural log of the factorial of a number

  @param n number

  @return Returns log( n! )
*/
static inline double log_factorial( int n )
{
  double f = 0;
  int i;

  for( i = 1; i <= n; i++ )
    f += log( i );

  return f;
}


/*
  Draws a RANSAC sample from a set of features.

  @param features array of pointers to features from which to sample
  @param n number of features in features
  @param m size of the sample

  @return Returns an array of pointers to the sampled features; the sampled
    field of each sampled feature's ransac_data is set to 1
*/
static struct feature** draw_ransac_sample( struct feature** features, int n,
					    int m )
{
  struct feature** sample, * feat;
  struct ransac_data* rdata;
  int i, x;

  for( i = 0; i < n; i++ )
    {
      rdata = feat_ransac_data( features[i] );
      rdata->sampled = 0;
    }

  sample = calloc( m, sizeof( struct feature* ) );
  for( i = 0; i < m; i++ )
    {
      do
	{
	  x = rand() % n;
	  feat = features[x];
	  rdata = feat_ransac_data( feat );
	}
      while( rdata->sampled );
      sample[i] = feat;
      rdata->sampled = 1;
    }

  return sample;
}



/*
  Extrancs raw point correspondence locations from a set of features

  @param features array of features from which to extract points and match
    points; each of these is assumed to have a match of type mtype
  @param n number of features
  @param mtype match type; if FEATURE_MDL_MATCH correspondences are assumed
    to be between each feature's img_pt field and it's match's mdl_pt field,
    otherwise, correspondences are assumed to be between img_pt and img_pt
  @param pts output as an array of raw point locations from features
  @param mpts output as an array of raw point locations from features' matches
*/
static void extract_corresp_pts( struct feature** features, int n, int mtype,
				 CvPoint2D64f** pts, CvPoint2D64f** mpts )
{
  struct feature* match;
  CvPoint2D64f* _pts, * _mpts;
  int i;

  _pts = calloc( n, sizeof( CvPoint2D64f ) );
  _mpts = calloc( n, sizeof( CvPoint2D64f ) );

  if( mtype == FEATURE_MDL_MATCH )
    for( i = 0; i < n; i++ )
      {
	match = get_match( features[i], mtype );
	if( ! match )
	  fatal_error( "feature does not have match of type %d, %s line %d",
		       mtype, __FILE__, __LINE__ );
	_pts[i] = features[i]->img_pt;
	_mpts[i] = match->mdl_pt;
      }

  else
    for( i = 0; i < n; i++ )
      {
	match = get_match( features[i], mtype );
	if( ! match )
	  fatal_error( "feature does not have match of type %d, %s line %d",
		       mtype, __FILE__, __LINE__ );
	_pts[i] = features[i]->img_pt;
	_mpts[i] = match->img_pt;
      }

  *pts = _pts;
  *mpts = _mpts;
}



/*
  For a given model and error function, finds a consensus from a set of
  feature correspondences.

  @param features set of pointers to features; every feature is assumed to
    have a match of type mtype
  @param n number of features in features
  @param mtype determines the match field of each feature against which to
    measure error; if this is FEATURE_MDL_MATCH, correspondences are assumed
    to be between the feature's img_pt field and the match's mdl_pt field;
    otherwise matches are assumed to be between img_pt and img_pt
  @param M model for which a consensus set is being found
  @param err_fn error function used to measure distance from M
  @param err_tol correspondences within this distance of M are added to the
    consensus set
  @param consensus output as an array of pointers to features in the
    consensus set

  @return Returns the number of points in the consensus set
*/
static int find_consensus( struct feature** features, int n, int mtype,
			   CvMat* M, ransac_err_fn err_fn, double err_tol,
			   struct feature*** consensus )
{
  struct feature** _consensus;
  struct feature* match;
  CvPoint2D64f pt, mpt;
  double err;
  int i, in = 0;

  _consensus = calloc( n, sizeof( struct feature* ) );

  if( mtype == FEATURE_MDL_MATCH )
    for( i = 0; i < n; i++ )
      {
	match = get_match( features[i], mtype );
	if( ! match )
	  fatal_error( "feature does not have match of type %d, %s line %d",
		       mtype, __FILE__, __LINE__ );
	pt = features[i]->img_pt;
	mpt = match->mdl_pt;
	err = err_fn( pt, mpt, M );
	if( err <= err_tol )
	  _consensus[in++] = features[i];
      }

  else
    for( i = 0; i < n; i++ )
      {
	match = get_match( features[i], mtype );
	if( ! match )
	  fatal_error( "feature does not have match of type %d, %s line %d",
		       mtype, __FILE__, __LINE__ );
	pt = features[i]->img_pt;
	mpt = match->img_pt;
	err = err_fn( pt, mpt, M );
	if( err <= err_tol )
	  _consensus[in++] = features[i];
      }
  *consensus = _consensus;
  return in;
}



/*
  Releases memory and reduces code size above

  @param pts1 an array of points
  @param pts2 an array of points
  @param features an array of pointers to features; can be NULL
*/
static inline void release_mem( CvPoint2D64f* pts1, CvPoint2D64f* pts2,
				struct feature** features )
{
  free( pts1 );
  free( pts2 );
  if( features )
    free( features );
}
