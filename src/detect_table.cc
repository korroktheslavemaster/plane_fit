#include <stdio.h>
#include <vector>

#include <Eigen/Dense>

#include "plane_fit.h"
#include <math.h>

namespace Constants {

// shoudl make these parameters that are changeable

  const double x_min = -1.0;
  const double x_max = 1.0;
  const double x_step = 0.1;
  const double y_min = -0.5;
  const double y_max = 0.5;
  const double y_step = 0.1;
  const double theta_min = -M_PI/4;
  const double theta_max = M_PI/4;
  const double theta_step = M_PI/20.;
}

using namespace Eigen;
using namespace Constants;

// returns %age of points that lie on the table specified by transform.
// assumes plane_points are on the correct plane
double percentPointsOnTable(const vector<SpaceCoord> &plane_points, 
  Pose table_pose, double x_half_len, double y_half_len) {
  // take the inverse transform of table_pose
  Pose table_inv = table_pose.inverse(Eigen::Affine);
  int count = 0;
  for (int i = 0; i < plane_points.size(); i++) {
    // apply inverse transform
    SpaceCoord pt = table_inv * plane_points[i];
    // make sure w = 1
    assert(pt[3] == 1.0);
    // check if it lies on the xy plane within the half lens
    count += (pt[0] <= x_half_len && pt[0] >= -x_half_len && 
      pt[1] <= y_half_len && pt[1] >= -y_half_len);    
  }
  return count/(double)plane_points.size();
}

Pose computePoseFromShifts(double x, double y, double theta, Pose base_pose) {
  // find the translation matrix
  Pose T;
  T.setIdentity();
  T.translation() = Eigen::Vector3d(x, y, 0);

  // find the rotation matrix
  Pose R(AngleAxisd(theta, Eigen::Vector3d(0,0,1)));

  return base_pose * T * R;
}

// takes a starting table pose, and generates poses around it.
vector<Pose> generateTablePoses(Pose base_pose, double x_min, double x_max, 
  double x_step, double y_min, double y_max, double y_step, double theta_min, 
  double theta_max, double theta_step) {
  // assertions to make sure min/max/step are sane
  assert(x_min <= x_max && y_min <= y_max && theta_min <= theta_max);
  assert(x_step > 0 && y_step > 0 && theta_step > 0);

  vector<Pose> results;
  for (double x = x_min; x <= x_max; x += x_step) {
    for (double y = y_min; y <= y_max; y += y_step) {
      for (double theta = theta_min; theta <= theta_max; theta += theta_step) {
        results.push_back(computePoseFromShifts(x, y, theta, base_pose));
      }
    }
  }
  return results;
}


Pose getBestTable(const Pose base_pose, const vector<SpaceCoord> &
  plane_points, double x_half_len, double y_half_len) {
  vector<Pose> table_poses = generateTablePoses(base_pose, x_min, x_max,
    x_step, y_min, y_max, y_step, theta_min, theta_max, theta_step);
  assert(table_poses.size() > 0);
  printf("Num of table poses = %d\n", (int)table_poses.size());
  Pose bestPose = table_poses[0];
  double bestPoseScore = percentPointsOnTable(plane_points, bestPose, 
    x_half_len, y_half_len);
  for (int i = 1; i < table_poses.size(); i++) {
    double score = percentPointsOnTable(plane_points, table_poses[i], 
      x_half_len, y_half_len);
    if (score > bestPoseScore) {
      bestPoseScore = score;
      bestPose = table_poses[i];
    }
  }
  printf("Best score = %lf\n", bestPoseScore*100);
  return bestPose;
}

// trying optimization technique.
// params: x shift, y shift, theta shift
// cost function: % points on table

#include <gsl/gsl_multimin.h>

struct OptParams {
  const Pose base_pose;
  const vector<SpaceCoord> *plane_points;
  double x_half_len, y_half_len;
  OptParams(const Pose base_pose, const vector<SpaceCoord> *plane_points, double x_half_len,
    double y_half_len): base_pose(base_pose), plane_points(plane_points), 
    x_half_len(x_half_len), y_half_len(y_half_len) {}
  OptParams(): base_pose(), plane_points(NULL), x_half_len(0), y_half_len(0) {}
};

double f(const gsl_vector * x, void *params_) {
  OptParams *params = static_cast<OptParams*>(params_);

  Pose pose = computePoseFromShifts(gsl_vector_get(x, 0), gsl_vector_get(x, 1), 
    gsl_vector_get(x, 2), params->base_pose);

  // negating since this is a minimizer
  return -percentPointsOnTable(*(params->plane_points), pose, params->x_half_len,
    params->y_half_len);
}

Pose getBestTableOpt(const Pose base_pose, const vector<SpaceCoord> &
  plane_points, double x_half_len, double y_half_len) {
  OptParams params(base_pose, &plane_points, x_half_len, y_half_len);
  const gsl_multimin_fminimizer_type *T =
  gsl_multimin_fminimizer_nmsimplex2;
  gsl_multimin_fminimizer *s = NULL;
  gsl_vector *ss, *x;
  gsl_multimin_function minex_func;
  size_t iter = 0;
  int status;
  double size;

  // 3 dim opt on x, y, theta
  int n = 3; 

  // set starting vector to all zeros
  x = gsl_vector_alloc(n);
  gsl_vector_set_all(x, 0);

  // set initial step sizes
  ss = gsl_vector_alloc(n);
  gsl_vector_set(ss, 0, x_step);
  gsl_vector_set(ss, 1, y_step);
  gsl_vector_set(ss, 2, theta_step);

  /* Initialize method and iterate */
  minex_func.n = n;
  minex_func.f = f;
  minex_func.params = &params;

  s = gsl_multimin_fminimizer_alloc (T, n);
  gsl_multimin_fminimizer_set (s, &minex_func, x, ss);

  do
  {
    iter++;
    status = gsl_multimin_fminimizer_iterate(s);

    if (status)
      break;

    size = gsl_multimin_fminimizer_size (s);
    status = gsl_multimin_test_size (size, 1e-2);

    if (status == GSL_SUCCESS)
    {
      // printf ("converged to minimum at\n");
    }

    // printf ("%5d %10.3e %10.3e f() = %7.3f size = %.3f\n",
    //         iter,
    //         gsl_vector_get (s->x, 0),
    //         gsl_vector_get (s->x, 1),
    //         s->fval, size);
  }
  while (status == GSL_CONTINUE && iter < 100);

  // s has the answer
  Pose pose = computePoseFromShifts(gsl_vector_get(s->x, 0), 
    gsl_vector_get(s->x, 1), gsl_vector_get(s->x, 2), base_pose);

  gsl_vector_free(x);
  gsl_vector_free(ss);  
  gsl_multimin_fminimizer_free (s);

  return pose;
}