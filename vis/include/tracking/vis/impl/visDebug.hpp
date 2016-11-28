/*
 * Created by: Aron Monszpart (aron.monszpart@gmail.com)
 * On: 9/3/2015
 *
 * Property of Adobe Inc.
*/

#ifndef SOUP_VISDEBUG_HPP
#define SOUP_VISDEBUG_HPP

#include "soup/modelTypes.h" // MeshType
#include "soup/visualization/visualizer.h"

namespace Soup
{
/*! \brief Visualization class for debugging. */
class VisDebug
{
    public:
        static void showCorresp( const ModelType& model0, const ModelType& model1, const CorrListType& corr, const float feature_radius );
        static void showCorresp( const ModelType& model0, const ModelType& model1, const CorrGroups& corrs, const float feature_radius );
        static void showCorresp( const MeshType& mesh0, const MeshType& mesh1_arg, const CorrGroups& corrs, const VerticesListType* keypointVertices0 = nullptr, const VerticesListType* keypointVertices1 = nullptr, const float feature_radius = 0.f, const int mod = 100 );
        template <typename _ModelPtrsT>
        static void showMatch  ( const MatchType& match, const _ModelPtrsT & models
            , const std::string title = "showMatch"
            , const typename Joint<geometry::Scalar>::ConstPtr& gtJoint = nullptr
            , const bool noKeypoints = false );
};
}

namespace Soup
{
inline void VisDebug::showCorresp( const ModelType& model0, const ModelType& model1, const CorrListType& corr, const float feature_radius )
{
    showCorresp( model0.getMesh(), model1.getMesh(), { corr }, &model0.getKeypointsVertices(), &model1.getKeypointsVertices(), feature_radius );
}
/*! \todo Move to cpp */
inline void VisDebug::showCorresp( const ModelType& model0, const ModelType& model1, const CorrGroups& corrs, const float feature_radius )
{
    showCorresp( model0.getMesh(), model1.getMesh(), corrs, &model0.getKeypointsVertices(), &model1.getKeypointsVertices(), feature_radius );
}

/*! \todo Move to cpp */
inline void VisDebug::showCorresp( const MeshType& mesh0, const MeshType& mesh1_arg, const CorrGroups& corrs, const VerticesListType* keypointVertices0, const VerticesListType* keypointVertices1, const float feature_radius, const int mod )
{
    typedef          geometry::Scalar                       Scalar;
    typedef typename geometry::HelperTypes<Scalar>::Vector3 Vector3;

    vis::Visualizer<Scalar> vis( "showCorresp" );
    vis.setBackgroundColor( 1., 1., 1. );
    vis.addMesh( mesh0, nullptr, "model0" );

    const VecType offset( 2., 0., 0. );
    QUASAR::Mesh mesh1 = mesh1_arg;
    for ( size_t i = 0; i != mesh1.getVertices().size(); ++i )
        mesh1.getVertex(i) += offset;

    vis.addMesh( mesh1, nullptr, "model1" );
    size_t groupId( 0 );
    for ( const CorrListType& corr : corrs )
    {
        ColorUtil::EigenColorMap color = ColorUtil::getInstance().getEigenColor( groupId );
        size_t corrId( 0ul );
        for ( auto it = corr.begin(); it != corr.end(); ++it, ++corrId )
        {
            if ( std::rand() / static_cast<float>(RAND_MAX) > (1. / mod) )
                continue;

            char lineName[255];
            sprintf( lineName, "group%04lu_corr%04lu", groupId, corrId );
            if ( keypointVertices0 && keypointVertices1 )
            {
                Vector3 end = (Soup::VecConstMap( (*keypointVertices1)[it->getTarget()].data() ) + Soup::VecConstMap(offset.data())).cast<Scalar>();
                vis.addLine( Soup::VecConstMap( (*keypointVertices0)[it->getSource()].data() ).cast<Scalar>()
                    , end
                    , color
                    , lineName );
            }
            else
            {
                vis.addLine   ( Soup::VecConstMap( mesh0.getVertices()[it->getSource()].data() ).cast<Scalar>()
                    , Soup::VecConstMap( mesh1.getVertices()[it->getSource()].data() ).cast<Scalar>()
                    , color
                    , lineName
                );
            } //...if no keypoints
        } //...for each correspondondence

        ++groupId;
    } //...for each group

    if ( feature_radius > 0.f )
        vis.addSphere( Vector3::Zero(), feature_radius, Vector3::Ones() );
    vis.spin();
} //...showCorresp()

template <typename _ModelPtrsT>
inline void VisDebug::showMatch( const MatchType& match, const _ModelPtrsT & models
    , const std::string title
    , const typename Joint<geometry::Scalar>::ConstPtr& gtJoint
    , const bool noKeypoints )
{
    typedef typename geometry::Scalar                                Scalar;
    typedef typename vis::Visualizer<Scalar>                         VisualizerT;
    typedef typename VisualizerT::Vector3                            Vector3;
    typedef typename geometry::ColoredCloudPtrT                      ColoredCloudPtrT;
    typedef typename ColoredCloudPtrT::element_type                  ColoredCloudT;
    typedef typename geometry::HelperTypes<Scalar>::Transform        Transform;


    const ModelType& model0( *models.at(match.getFirstUniqueId ()) ),
                     model1( *models.at(match.getSecondUniqueId()) );
    ColoredCloudPtrT cloud0, cloud1;
    ColoredCloudT::fromMesh( model0.getMesh(), cloud0 );
    ColoredCloudT::fromMesh( model1.getMesh(), cloud1 );

    typename VisualizerT::Ptr v( new VisualizerT( title ) );
    //v->addPointCloudMono( cloud0.getPoints(), Vector3(1.,1.,1.), "cloud0" ); // cloud1


    cloud1->transformCloud( Transform( Eigen::Translation<Scalar,3>(Vector3(0.,0.,-1.)) ), *cloud1 );
    cloud0->getColors().setZero();
    cloud1->getColors().setZero();

    ColorListType colors = ColorUtil::getInstance().getColors();
    for ( size_t corrGroupId = 0; corrGroupId != match.getCorrespondenceGroups().size(); ++corrGroupId )
    {
        const CorrListType& corrGroup = match.getCorrespondenceGroup( corrGroupId );
        for ( size_t corrId = 0; corrId != corrGroup.size(); ++corrId )
        {
            if ( corrGroupId >= colors.size() )
                std::cout << "bumm " << corrGroupId << "/" << colors.size() << std::endl;

            if ( noKeypoints )
            {
                cloud0->getColor( corrGroup[corrId].getSource() ) = Eigen::Map<const Eigen::Vector3f>( colors.at(corrGroupId % colors.size()).data() );
                cloud1->getColor( corrGroup[corrId].getTarget() ) = Eigen::Map<const Eigen::Vector3f>( colors.at(corrGroupId % colors.size()).data() );

                if ( !(corrId%250) )
                {
                    char name[255];
                    sprintf( name, "line_%ld_%ld", corrGroupId, corrId );
                    Eigen::Vector3f color( Eigen::Map<const Eigen::Vector3f>(colors.at(corrGroupId%colors.size()).data()) / 2. );
                    v->addLine( cloud0->getPoint(corrGroup[corrId].getSource())
                        , cloud1->getPoint(corrGroup[corrId].getTarget())
                        , color //, Eigen::Map<const Eigen::Vector3f>(colors.at(corrGroupId%colors.size()).data())
                        , name );
                }
            }
            else
            {
                if ( !(corrId%10))
                {
                    char name[255];
                    sprintf( name, "line_%ld_%ld", corrGroupId, corrId );
                    v->addLine( cloud0->getPoint(model0.getKeypointsIndices()[corrGroup[corrId].getSource()])
                        , cloud1->getPoint(model1.getKeypointsIndices()[corrGroup[corrId].getTarget()])
                        , Eigen::Map<const Eigen::Vector3f>(colors.at(corrGroupId%colors.size()).data()), name );
                }
                cloud0->getColor( model0.getKeypointsIndices()[corrGroup[corrId].getSource()] ) = Eigen::Map<const Eigen::Vector3f>( colors.at(corrGroupId % colors.size()).data() );
                cloud1->getColor( model1.getKeypointsIndices()[corrGroup[corrId].getTarget()] ) = Eigen::Map<const Eigen::Vector3f>( colors.at(corrGroupId % colors.size()).data() );
            }
        }
    }

    if ( match.getJoints().size() )
    {
        auto joint = match.getJoints().begin()->second;
        v->addLine( joint->getOrigin() - joint->getDirection(), joint->getOrigin() + joint->getDirection(), Vector3(0.,0.,1.), "jointLine" );
        v->addArrow( joint->getOrigin()- joint->getDirection()/2., joint->getOrigin() + joint->getDirection(), Vector3(0.,0.,1.), "jointArrow" );
        if ( gtJoint )
            v->addLine( gtJoint->getOrigin() - gtJoint->getDirection(), gtJoint->getOrigin() + gtJoint->getDirection(), Vector3(1.,.2,.2), "gt" );
    }
    v->addPointCloud( cloud0->getPoints(), cloud0->getColors(), "cloud0", &(cloud0->getNormals()) ); // cloud1
    v->setPointSize( 10., "cloud0" );
    v->addPointCloud( cloud1->getPoints(), cloud1->getColors(), "cloud1", &(cloud1->getNormals()) ); // cloud1
    v->setPointSize( 10., "cloud1" );
    v->setBackgroundColor( 1., 1., 1. );
    v->spin();
} //...showMatch()
}

#endif // SOUP_VISDEBUG_HPP
