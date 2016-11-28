#include <assert.h>
#include <stdbool.h>
#include <math.h>

#include "fail.c"
#include "xmalloc.c"

// generic function
// evaluate the error of a datapoint according to a model
// (implementing this function is necessary for each ransac case)
typedef float (ransac_error_evaluation_function)(
		float *model,
		float *datapoint,
		void *usr
		);


// generic function
// compute the model defined from a few data points
// (shall return 0 if no model could be computed)
// (implementing this function is necessary for each ransac case)
typedef int (ransac_model_generating_function)(
		float *out_model,  // parameters of the computed model
		float *data,       // data points
		void *usr
		);

// generic function
// tell whether a given model is good enough (e.g., not severely distorted)
// (this function is optional, and only serves as an optimization hint)
typedef bool (ransac_model_accepting_function)(
		float *model,
		void  *usr);


// API function: evaluate a given model over the data, and fill a mask with the
// inliers (according to the given allowed error).  This function returns the
// number of inliers.
int ransac_trial(
		// output
		bool *out_mask,    // array mask identifying the inliers

		// input data
		float *data,       // array of input data
		float *model,      // parameters of the model
		float max_error,   // maximum allowed error

		// input context
		int datadim,       // dimension of each data point
		int n,             // number of data points
		ransac_error_evaluation_function *mev,

		// decoration
		void *usr
		)
{
	int cx = 0;
	for (int i = 0; i < n; i++)
	{
		float *datai = data + i*datadim;
		float e = mev(model, datai, usr);
		if (!(e >= 0)) fprintf(stderr, "WARNING e = %g\n", e);
		assert(e >= 0);
		out_mask[i] = e < max_error;
		if (out_mask[i])
			cx += 1;
	}
	return cx;
}

// utility function: return a random number in the interval [a, b)
static int random_index(int a, int b)
{
	int r = a + rand()%(b - a);
	assert(r >= a);
	assert(r < b);
	return r;
}

// comparison function for the qsort call below
static int compare_ints(const void *aa, const void *bb)
{
	const int *a = (const int *)aa;
	const int *b = (const int *)bb;
	return (*a > *b) - (*a < *b);
}

// check whether a vector of n ints has different entries
// (sorts the vector inplace)
static bool are_different(int *t, int n)
{
	qsort(t, n, sizeof*t, compare_ints);
	for (int i = 1; i < n; i++)
		if (t[i-1] == t[i])
			return false;
	return true;
}

// generate a set of n different ints between a and b
static void fill_random_indices(int *idx, int n, int a, int b)
{
	if (b-a==n) {for(int i=0;i<n;i++)idx[i]=a+i;return;}
	//if (5*n > (b-a)) {fill_random_shuffle(idx, n, a, b);return;}
	// TODO fisher yates shuffle and traverse it by blocks of length nfit
	int safecount = 0;
	do {
		for (int i = 0; i < n; i++)
			idx[i] = random_index(a, b);
		safecount += 1;
	} while (safecount < 100 && !are_different(idx, n));
	if (safecount == 100)
		fail("could not generate any model (%d %d)", n, b-a);
}

#define MAX_MODELS 10


// RANSAC
//
// Given a list of data points, find the parameters of a model that fits to
// those points.  Several models are tried, and the model with the highest
// number of inliers is kept.
//
// A basic idea of this kind of ransac is that a maximum allowed error is fixed
// by hand, and then the inliers of a model are defined as the data points
// which fit the model up to the allowed error.  The RANSAC algorithm randomly
// tries several models and keeps the one with the largest number of inliers.
int ransac(
		// output
		//int *out_ninliers, // number of inliers
		bool *out_mask,    // array mask identifying the inliers
		float *out_model,  // model parameters

		// input data
		float *data,       // array of input data

		// input context
		int datadim,       // dimension of each data point
		int n,             // number of data points
		int modeldim,      // number of model parameters
		ransac_error_evaluation_function *mev,
		ransac_model_generating_function *mgen,
		int nfit,          // data points needed to produce a model

		// input parameters
		int ntrials,       // number of models to try
		int min_inliers,   // minimum allowed number of inliers
		float max_error,   // maximum allowed error

		// decoration
		ransac_model_accepting_function *macc,
		void *usr
		)
{

	int best_ninliers = 0;
	float best_model[modeldim];
	bool *best_mask = xmalloc(n * sizeof*best_mask);
	bool *tmp_mask = xmalloc(n * sizeof*best_mask);

	for (int i = 0; i < ntrials; i++)
	{
		int indices[nfit];
		fill_random_indices(indices, nfit, 0, n);

		float x[nfit*datadim];
		for (int j = 0; j < nfit; j++)
		for (int k = 0; k < datadim; k++)
			x[datadim*j + k] = data[datadim*indices[j] + k];

		float model[modeldim*MAX_MODELS];
		int nm = mgen(model, x, usr);
		if (!nm)
			continue;
		if (macc && !macc(model, usr))
			continue;

		// generally, nm=1
		for (int j = 0; j < nm; j++)
		{
			float *modelj = model + j*modeldim;
			int n_inliers = ransac_trial(tmp_mask, data, modelj,
					max_error, datadim, n, mev, usr);

			if (n_inliers > best_ninliers)
			{
				best_ninliers = n_inliers;
				for(int k = 0; k < modeldim; k++)
					best_model[k] = modelj[k];
				for(int k = 0; k < n; k++)
					best_mask[k] = tmp_mask[k];
			}
		}
	}

	int return_value = 0;
	if (best_ninliers >= min_inliers)
	{
		return_value =  best_ninliers;
	} else
		return_value = 0;

	for (int j = 0; j < modeldim; j++)
		if (!isfinite(best_model[j]))
			fail("model_%d not finite", j);

	if (out_model)
		for(int j = 0; j < modeldim; j++)
			out_model[j] = best_model[j];
	if (out_mask)
		for(int j = 0; j < n; j++)
			out_mask[j] = best_mask[j];

	free(best_mask);
	free(tmp_mask);

	return return_value;
}
