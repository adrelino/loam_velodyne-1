#ifndef OPENLOAM_H_
#define OPENLOAM_H_

// PCL includes
//#include <pcl/sample_consensus/ransac.h>


namespace openloam
{
  /** \brief @b openLOAM provides a base implementation of the Iterative Closest Point algorithm.
    * The transformation is estimated based on Singular Value Decomposition (SVD).
    *
    * The algorithm has several termination criteria:
    *
    * <ol>
    * <li>Number of iterations has reached the maximum user imposed number of iterations (via \ref setMaximumIterations)</li>
    * <li>The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via \ref setTransformationEpsilon)</li>
    * <li>The sum of Euclidean squared errors is smaller than a user defined threshold (via \ref setEuclideanFitnessEpsilon)</li>
    * </ol>
    *
    *
    * Usage example:
    * \code
    * IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    * // Set the input source and target
    * icp.setInputCloud (cloud_source);
    * icp.setInputTarget (cloud_target);
    *
    * // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    * icp.setMaxCorrespondenceDistance (0.05);
    * // Set the maximum number of iterations (criterion 1)
    * icp.setMaximumIterations (50);
    * // Set the transformation epsilon (criterion 2)
    * icp.setTransformationEpsilon (1e-8);
    * // Set the euclidean distance difference epsilon (criterion 3)
    * icp.setEuclideanFitnessEpsilon (1);
    *
    * // Perform the alignment
    * icp.align (cloud_source_registered);
    *
    * // Obtain the transformation that aligned cloud_source to cloud_source_registered
    * Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    * \endcode
    *
    * \author Sebastian Dingler
    * \ingroup registration
    */
  class openLOAM
  {
    public:

      /** \brief Empty constructor. */
      openLOAM ()
        //: x_idx_offset_ (0)
        //, y_idx_offset_ (0)
      {
        //reg_name_ = "IterativeClosestPoint";
        //transformation_estimation_.reset (new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar> ());
        //correspondence_estimation_.reset (new pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>);
        //convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<Scalar> (nr_iterations_, transformation_, *correspondences_));
      }

      /** \brief Empty destructor */
      ~openLOAM () {}

      /** \brief Provide a pointer to the input source
        * (e.g., the point cloud that we want to align to the target)
        *
        * \param[in] cloud the input point cloud source
        */
      void
      setInputCloud (const PointCloudSourceConstPtr &cloud);

      /** \brief Starts open loam */
      void
      run ();

    protected:

      /** \brief Checks for whether estimators and rejectors need various data */
      bool need_source_blob_, need_target_blob_;
  };

}

#endif  //#ifndef OPENLOAM_H_

