/*
 * Created by: Aron Monszpart (aron.monszpart@gmail.com)
 * On: 8/7/2015
 *
 * Property of Adobe Inc.
*/

#ifndef SOUP_RENDERER_HPP
#define SOUP_RENDERER_HPP

#include "soup/visualization/visualizer.h"
#include "soup/joints/hingeJoint.h"
#include "soup/joints/ballJoint.h"
#include "soup/joints/slidingHingeJoint.h"
#include "soup/joints/prismaticJoint.h"

namespace Soup
{
template <typename _Scalar>
class Renderer
{
    public:
        typedef typename vis::Visualizer<_Scalar>::Ptr  VisPtr;
        typedef typename Joint<_Scalar>::ConstPtr       JointConstPtr;
        typedef typename Joint<_Scalar>::Vector3        Vector3;
        typedef _Scalar Scalar;

        static void show( VisPtr vis, JointConstPtr              joint, const ScansTransforms& transforms, const Vector3& color, const std::string& name );
        static void show( VisPtr vis, const HingeJoint<_Scalar>& joint, const ScansTransforms& transforms, const Vector3& color, const std::string& name );
}; //...class Renderer
} //...ns Soup

namespace Soup
{
template <typename _Scalar>
inline void Renderer<_Scalar>::show( VisPtr vis, JointConstPtr joint, const ScansTransforms& transforms, const Vector3& color, const std::string& name )
{
    switch ( joint->getType() )
    {
        case Joint<_Scalar>::HINGE:
            show( vis, *std::dynamic_pointer_cast<const HingeJoint<_Scalar>>(joint), transforms, color, name );
            break;
        case Joint<_Scalar>::JOINT:
            break;
        default:
            std::cout << "[" << __func__ << "]: undefined show function" << std::endl;
            break;
    }
}

template <typename _Scalar>
inline void Renderer<_Scalar>::show( VisPtr vis, const HingeJoint<_Scalar>& joint, const ScansTransforms& transforms, const Vector3& color, const std::string& name )
{
    vis->addLine( joint.getOrigin() - joint.getDirection(), joint.getOrigin() + joint.getDirection(), color, name );

    vis->addPointCloud( transforms.getInputCloud(0)->getPoints(), transforms.getInputCloud(0)->getColors(), "input0");
    vis->addPointCloud( transforms.getInputCloud(1)->getPoints(), transforms.getInputCloud(1)->getColors(), "input1" );
    for ( int i = 0; i != transforms.size(); ++i )
        for ( int j = 0; j != transforms.getScan(i).size(); ++j )
        {
            char name[255];
            sprintf( name, "observed%02d%02d", i, j );
            vis->addPointCloud( transforms.getCloud(i,j)->getPoints(), transforms.getCloud(i,j)->getColors(), name );
        }
}
} //...Soup

#endif // SOUP_RENDERER_HPP
