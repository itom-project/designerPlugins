#ifndef POINTCLOUDHANDLERGENERICFIELDS_H
#define POINTCLOUDHANDLERGENERICFIELDS_H

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <vector>
#include <math.h>


//////////////////////////////////////////////////////////////////////////////////////
/** \brief Generic field handler class for colors. Uses an user given field to extract
    * 1D data and display the color at each point using a min-max lookup table.
    * \author Radu B. Rusu 
    * \ingroup visualization
    */
template <typename PointT>
class PointCloudColorHandlerGenericFields : public pcl::visualization::PointCloudColorHandler<PointT>
{
    typedef typename pcl::visualization::PointCloudColorHandler<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

protected:
    /** \brief Class getName method. */
    virtual std::string
    getName () const { return ("PointCloudColorHandlerGenericFields"); }

    private:
    using pcl::visualization::PointCloudColorHandler<PointT>::cloud_;
    using pcl::visualization::PointCloudColorHandler<PointT>::capable_;
    using pcl::visualization::PointCloudColorHandler<PointT>::fields_;

    /** \brief Name of the field used to create the color handler. */
    std::vector<std::string> field_names_;
    std::vector<int> field_idxs_;

public:
    typedef boost::shared_ptr<PointCloudColorHandlerGenericFields<PointT> > Ptr;
    typedef boost::shared_ptr<const PointCloudColorHandlerGenericFields<PointT> > ConstPtr;

    /** \brief Constructor. */
    PointCloudColorHandlerGenericFields (const std::string &field_name1, const std::string &field_name2, const std::string &field_name3)
    {
        field_names_.push_back(field_name1);
        field_names_.push_back(field_name2);
        field_names_.push_back(field_name3);
        capable_ = false;
    }

    /** \brief Constructor. */
    PointCloudColorHandlerGenericFields (const std::vector<std::string> &field_names) :
        field_names_(field_names)
    {
        capable_ = false;
    }

    /** \brief Constructor. */
    PointCloudColorHandlerGenericFields (const PointCloudConstPtr &cloud,
                                        const std::string &field_name1, 
                                        const std::string &field_name2, 
                                        const std::string &field_name3)
        : pcl::visualization::PointCloudColorHandler<PointT> (cloud)
    {
        field_names_.push_back(field_name1);
        field_names_.push_back(field_name2);
        field_names_.push_back(field_name3);
        setInputCloud (cloud);
    }

    /** \brief Constructor. */
    PointCloudColorHandlerGenericFields (const PointCloudConstPtr &cloud,
                                        const std::vector<std::string> &field_names)
        : pcl::visualization::PointCloudColorHandler<PointT> (cloud),
        field_names_(field_names)
    {
        setInputCloud (cloud);
    }

    /** \brief Destructor. */
    virtual ~PointCloudColorHandlerGenericFields () {}

    /** \brief Get the name of the field used. */
    virtual std::string getFieldName () const 
    { 
        if (field_names_.empty())
        {
            return "";
        }
        else
        {
            std::string result;
            for (int i = 0; i < field_names_.size() - 1; ++i)
            {
                result += field_names_[i];
                result += ";";
            }
            result += field_names_[field_names_.size()-1];
            return result;
        }
    }

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
        * \param[out] scalars the output scalars containing the color for the dataset
        * \return true if the operation was successful (the handler is capable and 
        * the input cloud was given as a valid pointer), false otherwise
        */
    virtual bool
    getColor (vtkSmartPointer<vtkDataArray> &scalars) const
    {
        if (!capable_ || !cloud_)
            return (false);

        if (!scalars)
            scalars = vtkSmartPointer<vtkFloatArray>::New ();
        scalars->SetNumberOfComponents (1);

        vtkIdType nr_points = cloud_->points.size ();
        reinterpret_cast<vtkFloatArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

        typedef typename pcl::traits::fieldList<PointT>::type FieldList;

        float* colors = new float[nr_points];
        float field_data;
        float fields_sum;

        int j = 0;
        // If XYZ present, check if the points are invalid
        int x_idx = -1;
        for (size_t d = 0; d < fields_.size (); ++d)
        if (fields_[d].name == "x")
            x_idx = static_cast<int> (d);

        if (x_idx != -1)
        {
        // Color every point
        for (vtkIdType cp = 0; cp < nr_points; ++cp)
        {
            // Copy the value at the specified field
            if (!std::isfinite(cloud_->points[cp].x) || !std::isfinite(cloud_->points[cp].y) || !std::isfinite(cloud_->points[cp].z))
            continue;

            const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud_->points[cp]);
            fields_sum = 0.0;
            for (int i = 0; i < field_names_.size(); ++i)
            {
                memcpy (&field_data, pt_data + fields_[field_idxs_[i]].offset, pcl::getFieldSize (fields_[field_idxs_[i]].datatype));
                fields_sum += field_data * field_data;
            }

            colors[j] = sqrtf(fields_sum);
            j++;
        }
        }
        else
        {
        // Color every point
        for (vtkIdType cp = 0; cp < nr_points; ++cp)
        {
            const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud_->points[cp]);
            fields_sum = 0.0;
            for (int i = 0; i < field_names_.size(); ++i)
            {
                memcpy (&field_data, pt_data + fields_[field_idxs_[i]].offset, pcl::getFieldSize (fields_[field_idxs_[i]].datatype));
                fields_sum += field_data * field_data;
            }

            if (!std::isfinite(field_data))
                continue;

            colors[j] =sqrtf(fields_sum);
            j++;
        }
        }
        reinterpret_cast<vtkFloatArray*>(&(*scalars))->SetArray (colors, j, 0, vtkFloatArray::VTK_DATA_ARRAY_DELETE);
        return (true);
    }

    /** \brief Set the input cloud to be used.
        * \param[in] cloud the input cloud to be used by the handler
        */
    virtual void
    setInputCloud (const PointCloudConstPtr &cloud)
    {
        pcl::visualization::PointCloudColorHandler<PointT>::setInputCloud (cloud);
        field_idxs_.clear();
        capable_ = (field_names_.size() > 0);
        for (int i = 0; i < field_names_.size(); ++i)
        {
            int idx = pcl::getFieldIndex (*cloud, field_names_[i], fields_);
            field_idxs_.push_back(idx);
            if (idx == -1)
                capable_ = false;
        }
    }


};

#endif
