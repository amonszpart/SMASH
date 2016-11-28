#ifndef SOUP_VIS_VISUALIZER_HPP
#define SOUP_VIS_VISUALIZER_HPP

#include "tracking/common/eigen.h"
#include "tracking/vis/visualizer.h"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkLODActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkPointSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointData.h>
#include <vtkAxes.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkTubeFilter.h>
#include <vtkFloatArray.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkLineSource.h>
#include <vtkMath.h>
#include <vtkArrowSource.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCamera.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkLightActor.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>

#include <chrono>
#include <sstream>
#include <vector>
#include <map>

namespace Soup {
namespace vis {
template <typename _Scalar>
inline void Visualizer<_Scalar>::init() {
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    vtkSmartPointer<vtkRenderer>     renderer     = vtkSmartPointer<vtkRenderer>::New();

#if 0
    double lightPosition[3] = {0, 0, 1};

            // Create a light
            double lightFocalPoint[3] = {0,0,0};

            vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
            light->SetLightTypeToSceneLight();
            light->SetPosition(lightPosition[0], lightPosition[1], lightPosition[2]);
            light->SetPositional(true); // required for vtkLightActor below
            light->SetConeAngle(10);
            light->SetFocalPoint(lightFocalPoint[0], lightFocalPoint[1], lightFocalPoint[2]);
            light->SetDiffuseColor(1,0,0);
            light->SetAmbientColor(0,1,0);
            light->SetSpecularColor(0,0,1);

            vtkLightCollection* originalLights = renderer->GetLights();

            //  renderer->AddLight(light); // can't do this here - must do this after the renderWindow->Render() below

            // Display where the light is
            //                 vtkSmartPointer<vtkLightActor> lightActor = vtkSmartPointer<vtkLightActor>::New();
            //                 lightActor->SetLight(light);
            //                 renderer->AddViewProp(lightActor);

            //                 // Display where the light is focused
            //                 vtkSmartPointer<vtkSphereSource> lightFocalPointSphere = vtkSmartPointer<vtkSphereSource>::New();
            //                 lightFocalPointSphere->SetCenter(lightFocalPoint);
            //                 lightFocalPointSphere->SetRadius(.1);
            //                 lightFocalPointSphere->Update();

            //                 vtkSmartPointer<vtkPolyDataMapper> lightFocalPointMapper =
            //                     vtkSmartPointer<vtkPolyDataMapper>::New();
            //                 lightFocalPointMapper->SetInputConnection(lightFocalPointSphere->GetOutputPort());

            //                 vtkSmartPointer<vtkActor> lightFocalPointActor = vtkSmartPointer<vtkActor>::New();
            //                 lightFocalPointActor->SetMapper(lightFocalPointMapper);
            //                 lightFocalPointActor->GetProperty()->SetColor(1.0, 1.0, 0.0); //(R,G,B)
            //                 renderer->AddViewProp(lightFocalPointActor);
#endif

    //                vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
    //                camera->SetPosition(-.25, .5, -.5);
    //                camera->SetFocalPoint(0, 0, 0);
    //                renderer->SetActiveCamera( camera );

    renderer->SetTwoSidedLighting(1);
    renderer->SetLightFollowCamera(1);
    renderWindow->AddRenderer(renderer);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
                                                   vtkSmartPointer<vtkRenderWindowInteractor>::New();

    _interactors.push_back(renderWindowInteractor);
    _renderers.push_back( renderer );
    _renderWindows.push_back( renderWindow );

    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderWindow->Render();
    renderWindow->SetWindowName( _name.c_str() );
    renderWindow->SetPosition( 10, 960 );
    renderWindow->SetSize( 960, 960 );
#if 0
    renderer->AddLight( light );
#endif

    vtkSmartPointer<KeyPressInteractorStyle> style =
                                                 vtkSmartPointer<KeyPressInteractorStyle>::New();
    //                vtkSmartPointer<vtkInteractorStyleRubberBandPick> style =
    //                        vtkSmartPointer<vtkInteractorStyleRubberBandPick>::New();
    renderWindowInteractor->SetInteractorStyle( style );
    style->SetCurrentRenderer(renderer);
    style->loadCamera();

    //renderWindow->SetPosition(i*300,0);

    //renderer->ResetCamera();
    exit_main_loop_timer_callback_ = vtkSmartPointer<ExitMainLoopTimerCallback>::New ();
    exit_main_loop_timer_callback_->visualizer = this;
    exit_main_loop_timer_callback_->right_timer_id = -1;
    exit_main_loop_timer_callback_->_interactor = _interactors.back();
    _interactors.back()->AddObserver (vtkCommand::TimerEvent, exit_main_loop_timer_callback_);

    this->setBackgroundColor( 1., 1., 1. );
} //...init()

template <typename _Scalar>
inline void Visualizer<_Scalar>::addSphere( const Visualizer<_Scalar>::Vector3& center, _Scalar radius, const Visualizer<_Scalar>::Vector3& colour, const std::string name )
{
    auto& renderer = _renderers.back();

    // create a sphere
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter( center(0), center(1), center(2) );
    sphereSource->SetRadius( radius );
    sphereSource->Update();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper( mapper );
    actor->GetProperty()->SetColor( colour(0), colour(1), colour(2) );

    if ( _actors.find(name) != _actors.end() )
        renderer->RemoveActor( actor );
    _actors[ name ] = actor;

    renderer->AddActor(actor);
} //...Visualizer::addSphere()

template <typename _Scalar>
inline void Visualizer<_Scalar>::addCylinder( const Visualizer<_Scalar>::Vector3& center
    , const Visualizer<_Scalar>::Vector3& dir
    , const _Scalar                       radius
    , const Visualizer<_Scalar>::Vector3& colour
    , const std::string                   name )
{
    auto& renderer = _renderers.back();

    // create a sphere
    vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSource->SetCenter( center(0), center(1), center(2) );
    cylinderSource->SetHeight( dir.norm() );
    cylinderSource->SetRadius( radius );
    cylinderSource->Update();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cylinderSource->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper( mapper );
    actor->GetProperty()->SetColor( colour(0), colour(1), colour(2) );
    auto v = dir.normalized();
    auto tau = std::atan2(v(1), -v(0));
    auto phi = std::asin (v(2));
    actor->RotateZ( tau * 180. / M_PI );
    actor->RotateX( phi * 180. / M_PI );
    std::cout << "tau: " << tau * 180. / M_PI << ", phi: " << phi * 180. / M_PI << std::endl;

    if ( _actors.find(name) != _actors.end() )
        renderer->RemoveActor( actor );
    _actors[ name ] = actor;

    renderer->AddActor(actor);
} //...Visualizer::addSphere()

/** \brief From PCL.
 */
template <typename _Scalar>
inline void Visualizer<_Scalar>::addCoordinateSystem( _Scalar scale, const std::string name )
{
    if ( !_interactors.size() )
        return;

    auto& renderWindowInteractor = _interactors[0];
    renderWindowInteractor->Render();

    if ( scale <= 0.0 )
        scale = 1.0;
    vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
    axes->SetOrigin (0, 0, 0);
    axes->SetScaleFactor (scale);
    axes->Update ();

    vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
    axes_colors->Allocate (6);
    axes_colors->InsertNextValue (0.0);
    axes_colors->InsertNextValue (0.0);
    axes_colors->InsertNextValue (0.5);
    axes_colors->InsertNextValue (0.5);
    axes_colors->InsertNextValue (1.0);
    axes_colors->InsertNextValue (1.0);

    vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
    axes_data->GetPointData ()->SetScalars (axes_colors);

    vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
#if VTK_MAJOR_VERSION < 6
    axes_tubes->SetInput (axes_data);
#else
    axes_tubes->SetInputData (axes_data);
#endif
    axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
    axes_tubes->SetNumberOfSides (6);

    vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    axes_mapper->SetScalarModeToUsePointData ();
#if VTK_MAJOR_VERSION < 6
    axes_mapper->SetInput (axes_tubes->GetOutput ());
#else
    axes_mapper->SetInputConnection (axes_tubes->GetOutputPort ());
#endif

    vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
    axes_actor->SetMapper (axes_mapper);

    if ( _actors.find(name) != _actors.end() )
        _renderers[0]->RemoveActor( axes_actor );
    _actors[ name ] = axes_actor;

    _renderers[0]->AddActor( axes_actor );
} //...addCoordinateSystem()

template <typename _Scalar>
inline void Visualizer<_Scalar>::addLine( const Visualizer<_Scalar>::Vector3 &p0, const Visualizer<_Scalar>::Vector3 &p1, const Visualizer<_Scalar>::Vector3& color, const std::string name, const Scalar lineWidth, const Scalar opacity )
{
    vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1( const_cast<Scalar*>(p0.data()) );
    lineSource->SetPoint2( const_cast<Scalar*>(p1.data()) );
    lineSource->Update();

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(lineSource->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper( mapper );
    actor->GetProperty()->SetLineWidth( lineWidth );
    actor->GetProperty()->SetColor( color(0), color(1), color(2) );
    actor->GetProperty()->SetOpacity( opacity );

    if ( removeActor(name) )
    {
        std::cout << "[" << __func__ << "]: " << "removing " << name << std::endl;
    }
    _actors[ name ] = actor;
    _renderers[0]->AddActor( actor );
} //...addLine()
#if 0
template <typename _Scalar>
        inline void Visualizer<_Scalar>::addLine( const VecType &p0, const VecType &p1, const Vector3& color, const std::string name, const _Scalar lineWidth, const _Scalar opacity )
        {
            addLine( Vector3(p0[0],p0[1],p0[2]), Vector3(p1[0],p1[1],p1[2]), color, name );
//            addLine( Eigen::Map< const Eigen::Matrix<typename VecType::Scalar, 3, 1> >(p0.data()).cast<Scalar>()
//                   , Eigen::Map< const Eigen::Matrix<typename VecType::Scalar, 3, 1> >(p1.data()).cast<Scalar>(), colour, name, lineWidth, opacity );
        } // addLine()
#endif

template <typename _Scalar>
inline void Visualizer<_Scalar>::spin()
{
    //std::cout << "[" << __func__ << "]: spin" << std::endl; fflush(stdout);
    for ( auto& renderWindow : _renderWindows )
    {
        renderWindow->Render();
        renderWindow->GetInteractor()->Render();
    }
    if ( _interactors.size() )
    {
        _interactors.back()->Start();
    }
} //...spin()

template <typename _Scalar>
inline void Visualizer<_Scalar>::reRender()
{
    for ( auto& renderWindow : _renderWindows )
    {
        renderWindow->Render();
        renderWindow->GetInteractor()->Render();
    }
} //...reRender()

template <typename _Scalar>
inline bool Visualizer<_Scalar>::removeActor( const std::string& name )
{
    auto it = _actors.find( name );
    if ( it == _actors.end() )
        return false;
    else
    {
        _renderers[0]->RemoveActor(it->second);
        _actors.erase(it);
        return true;
    }
}

template <typename _Scalar>
inline bool Visualizer<_Scalar>::hasActor(std::string const& name) const {
    return _actors.find(name) != _actors.end();
} //...Visualizer::hasActor()

template <typename _Scalar>
inline bool Visualizer<_Scalar>::removeActorsByPrefix(std::string const& prefix)
{
    for ( auto it = _actors.begin(); it != _actors.end(); )
    {
        if ( it->first.compare(0, prefix.size(), prefix) == 0 )
        {
            _renderers[0]->RemoveActor(it->second);
            it = _actors.erase( it );
        }
        else
            ++it;
    }
    return true;
} //...removeActorsByPrefix()

template <typename _Scalar>
inline bool Visualizer<_Scalar>::removeActorsByPrefix(std::vector<std::string> const& prefixes) {
    for (auto it = _actors.begin(); it != _actors.end();) {
        if (std::find_if(prefixes.begin(), prefixes.end(), [&it](std::string const& prefix){
            return it->first.compare(0, prefix.size(), prefix) == 0;
        }) != prefixes.end()) {
            _renderers[0]->RemoveActor(it->second);
            it = _actors.erase(it);
        }
        else
            ++it;
    } //...for all actors
    return true;
} //...removeActorsByPrefix()

template <typename _Scalar>
inline bool Visualizer<_Scalar>::removeActorsAll()
{
    for ( auto it = _actors.begin(); it != _actors.end(); ++it)
    {
        _renderers[0]->RemoveActor(it->second);
    }
    _actors.clear();

    return true;
} //...removeActorsByPrefix()

#if 1
// from pcl: https://github.com/PointCloudLibrary/pcl/blob/e8a88803bc5ffdabaf366e02f7e2f72fcf7fd4fb/common/include/pcl/common/time.h
inline double
getTime ()
{
    //return std::chrono::duration_cast<std::chrono::seconds>( std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) ).count();
    return std::chrono::duration_cast<std::chrono::seconds>( std::chrono::system_clock::now().time_since_epoch() ).count();
}

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code) \
        if (1) {\
          static double s_lastDone_ = 0.0; \
          double s_now_ = (currentTime); \
          if (s_lastDone_ > s_now_) \
            s_lastDone_ = s_now_; \
          if ((s_now_ - s_lastDone_) > (secs)) {        \
            code; \
            s_lastDone_ = s_now_; \
          }\
        } else \
          (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) \
          DO_EVERY_TS(secs, getTime(), code)
#endif

template <typename _Scalar>
inline void Visualizer<_Scalar>::spinOnce( int time )
{
    if (time <= 0)
        time = 1;

//              if (force_redraw)
//                interactor_->Render ();
    reRender();
    DO_EVERY (1.0 / _interactors.back()->GetDesiredUpdateRate (),
              exit_main_loop_timer_callback_->right_timer_id = _interactors.back()->CreateRepeatingTimer (time);
                  _interactors.back()->Start ();
                  _interactors.back()->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
    );
} //...spinOnce()
template <typename _Scalar>
void Visualizer<_Scalar>::ExitMainLoopTimerCallback::Execute (
    vtkObject*, unsigned long event_id, void* call_data)
{
    if (event_id != vtkCommand::TimerEvent)
        return;
    int timer_id = * static_cast<int*> (call_data);
    if (timer_id != right_timer_id)
        return;
    // Stop vtk loop and send notification to app to wake it up
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
    _interactor->stopLoop ();
#else
    _interactor->TerminateApp();
#endif
}
#endif

template <typename _Scalar>
inline void Visualizer<_Scalar>::addCube( const Vector3& pos, const Vector3& size, const Vector3& colour, const std::string& name )
{
    auto& renderer = _renderers.back();

    // create a sphere
    vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
    cubeSource->SetBounds( pos(0) - size(0)/2., pos(0) + size(0)/2.,
                           pos(1) - size(1)/2., pos(1) + size(1)/2.,
                           pos(2) - size(2)/2., pos(2) + size(2)/2.
    );
    cubeSource->Update();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cubeSource->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper( mapper );
    actor->GetProperty()->SetColor( colour(0), colour(1), colour(2) );
    actor->GetProperty()->SetRepresentationToWireframe();

    if ( _actors.find(name) != _actors.end() )
        renderer->RemoveActor( actor );
    _actors[ name ] = actor;

    renderer->AddActor(actor);
} //...addCube()

template <typename _Scalar>
inline void Visualizer<_Scalar>::addPointCloudMono( const Visualizer<_Scalar>::CloudT& vertices
    , const Visualizer<_Scalar>::Vector3& colour
    , const std::string name
    , const Visualizer<_Scalar>::CloudT* normals )
{
    this->addPointCloud( vertices, (Visualizer<_Scalar>::CloudT(1,3) << colour.transpose()).finished(), name, normals );
}

template <typename _Scalar>
inline void Visualizer<_Scalar>::addPointCloud( const ColoredPointCloud<_Scalar>& cloud, const std::string& name) {
    if ( cloud.hasNormals() )
        this->addPointCloud( cloud.getPoints(), cloud.getColors(), name, &cloud.getNormals() );
    else
        this->addPointCloud( cloud.getPoints(), cloud.getColors(), name );
}

template <typename _Scalar>
template <typename _Derived, typename _DerivedB>
inline void Visualizer<_Scalar>::addPointCloud( const Eigen::MatrixBase<_Derived> & vertices //const Visualizer<_Scalar>::CloudT& vertices
    , const Eigen::MatrixBase<_DerivedB>& colors
    , const std::string                   name
    , const Visualizer<_Scalar>::CloudT  * normals )
{
    if ( (vertices.rows() != colors.rows()) && (colors.rows() > 1) )
    {
        std::cerr << "[" << __func__ << "]: same number of vertices and colours are expected: " << vertices.rows() << " vs. " << colors.rows() << std::endl;
        return;
    }

    const size_t               N     ( normals ? 2 * vertices.rows() : vertices.rows() );
    vtkSmartPointer<vtkPoints> points( vtkSmartPointer<vtkPoints>::New() );
    points->SetNumberOfPoints( N );
    vtkSmartPointer<vtkFloatArray> pointNormalsArray(nullptr);
    if ( normals )
    {
        pointNormalsArray = vtkSmartPointer<vtkFloatArray>::New();
        pointNormalsArray->SetNumberOfComponents( 3 );
        pointNormalsArray->SetNumberOfTuples( N );

        size_t                     pid2( 0 );
        Eigen::Matrix<_Scalar,1,3> tmpPoint;
        Eigen::Matrix<float,1,3>   tmpNormal;
        for ( size_t pid = 0; pid != size_t(vertices.rows()); ++pid )
        {
            points->GetData()->SetTuple( pid2, vertices.row(pid).data() );
            tmpNormal = normals->getPoint(pid).template cast<float>();
            pointNormalsArray->SetTuple( pid2++, tmpNormal.data() );

            tmpNormal *= -1.f;
            tmpPoint   = vertices.row(pid).template cast<_Scalar>() + tmpNormal * 0.001f;
            points->GetData()->SetTuple( pid2  , tmpPoint.data()       );
            pointNormalsArray->SetTuple( pid2++, tmpNormal.data() );
        } //...for points
    } //...if normals
    else
    {
        for ( int pid = 0; pid != vertices.rows(); ++pid )
        {
            points->GetData()->SetTuple( pid, vertices.row(pid).data() );
        } //...for points
    } //...no normals

    // Add points
    vtkSmartPointer<vtkPolyData> pointsPolyData = vtkSmartPointer<vtkPolyData>::New();
    pointsPolyData->SetPoints( points );
    if ( normals )
    {
        // Add the normals to the points in the polydata
        pointsPolyData->GetPointData()->SetNormals( pointNormalsArray );
    } //...if normals

    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    vertexFilter->SetInputConnection(pointsPolydata->GetProducerPort());
#else
    vertexFilter->SetInputData(pointsPolyData);
#endif
    vertexFilter->Update();

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->ShallowCopy(vertexFilter->GetOutput());

    // Colors
    if ( colors.rows() )
    {
        vtkSmartPointer<vtkUnsignedCharArray> colorsVtk = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colorsVtk->SetNumberOfComponents( 3 );
        colorsVtk->SetNumberOfTuples    ( pointsPolyData->GetNumberOfPoints() );
        colorsVtk->SetName              ( "Colors" );
        unsigned char color[3];
        size_t pid2( 0 );
        for ( size_t pid = 0; pid != size_t(vertices.rows()); ++pid )
        {
            (Eigen::Map< Eigen::Matrix<unsigned char,3,1> >(color)) =
                (colors.row( colors.rows() > 1 ? pid : 0) * 255.0).template cast<unsigned char>();
            //colors->InsertNextTupleValue( c );
            colorsVtk->SetTupleValue( pid2++, color );
            if ( normals && pid2 < static_cast<size_t>(colorsVtk->Capacity()) )
                colorsVtk->SetTupleValue( pid2++, color );
        }
        polydata->GetPointData()->SetScalars( colorsVtk );
    }

    // Visualization
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
    mapper->SetInputConnection(polydata->GetProducerPort());
#else
    mapper->SetInputData(polydata);
#endif

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(3);
    actor->GetProperty()->SetInterpolation( VTK_GOURAUD );

    vtkSmartPointer<vtkRenderer> renderer = _renderers.back();

    if ( _actors.find(name) != _actors.end() )
        renderer->RemoveActor( _actors[name] );
    _actors[ name ] = actor;
    renderer->AddActor(actor);
} //...addPointCloud()

template <typename _Scalar>
inline void Visualizer<_Scalar>::setPointSize( double size, std::string actorName )
{
    auto it = _actors.find( actorName );
    if ( it != _actors.end() )
        it->second->GetProperty()->SetPointSize( size );
}

template <typename _Scalar>
template <typename _MeshT>
inline void Visualizer<_Scalar>::addMesh( const _MeshT &mesh, const Vector3 color, const std::string name, const int withNormals )
{
    addMesh( mesh, &color, name, withNormals );
} //...addMesh()

template <typename _Scalar>
template <typename _MeshT>
inline void Visualizer<_Scalar>::addMesh( const _MeshT &mesh, const Vector3* color, const std::string name, const int withNormals )
{
    typename PointCloud<_Scalar>::Ptr cloud, colors, normals;
    ColoredPointCloud<_Scalar>::fromMesh( mesh, cloud, colors, normals );
    //std::cout << "[" << __func__ << "]: " << name << "[12982]: " << cloud->getPoint( 12982 ) << std::endl;

    if ( color )
        this->addPointCloudMono( *cloud, *color, name, normals.get() );
    else if ( colors->getNPoints() )
        this->addPointCloud( *cloud, *colors, name, normals.get() );
    else
        this->addPointCloudMono( *cloud, Vector3::Ones(), name, normals.get() );

    if ( withNormals )
    {
        Vector3 normalColor( Vector3::Ones() );
        if ( color )
        {
            normalColor = *color * 0.7;
        }

        char lineName[255];
        for ( size_t i = 0; i < mesh.getNormals().size(); i += withNormals )
        {
            sprintf( lineName, "mesh%sline%03lu", name.c_str(), i );
            this->addLine( mesh.getVertex(i), mesh.getVertex(i) + mesh.getNormal(i) * 0.025, normalColor, lineName );
        }
    }
} //...addMesh()

template <typename _Scalar>
inline void Visualizer<_Scalar>::setCameraFocalPoint(Eigen::Vector3d const& focal, int const renderer) {
    _renderers.at(renderer)->GetActiveCamera()->SetFocalPoint(focal.data());
} //...setCameraFocalPoint()

template <typename _Scalar>
inline void Visualizer<_Scalar>::setCameraViewUp(Eigen::Vector3d const& up, int const renderer) {
    _renderers[renderer]->GetActiveCamera()->SetViewUp(up.data());
} //...setCameraViewUp()

template <typename _Scalar>
inline Eigen::Map<const Eigen::Vector3d> Visualizer<_Scalar>::getCameraViewUp(int const renderer) {
    return Eigen::Map<const Eigen::Vector3d>{_renderers[renderer]->GetActiveCamera()->GetViewUp()};
}

template <typename _Scalar>
inline Eigen::Map<const Eigen::Vector3d> Visualizer<_Scalar>::getCameraPosition(int const renderer) {
    return Eigen::Map<const Eigen::Vector3d>{_renderers[renderer]->GetActiveCamera()->GetPosition() };
}

template <typename _Scalar>
inline Eigen::Map<const Eigen::Vector3d> Visualizer<_Scalar>::getCameraFocalPoint(int const renderer) {
    return Eigen::Map<const Eigen::Vector3d>{_renderers[renderer]->GetActiveCamera()->GetFocalPoint()};
}

// http://www.vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/OrientedArrow
template <typename _Scalar>
inline void
Visualizer<_Scalar>::addArrow(const Vector3& p0, const Vector3& p1, const Vector3& color, const std::string name,
                              Scalar thickness)
{
    //Create an arrow.
    vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();

    // Compute a basis
    double normalizedX[3], normalizedY[3], normalizedZ[3];

    Eigen::Vector3d p0p1( (p1-p0).template cast<double>() );
    double length = p0p1.norm();
    (Eigen::Map<Eigen::Vector3d>(normalizedX)) = p0p1 / length;

    // The Z axis is an arbitrary vector cross X
    double arbitrary[3];
    arbitrary[0] = vtkMath::Random(-10.,10.);
    arbitrary[1] = vtkMath::Random(-10.,10.);
    arbitrary[2] = vtkMath::Random(-10.,10.);
    vtkMath::Cross( normalizedX, arbitrary, normalizedZ );
    vtkMath::Normalize( normalizedZ );

    // The Y axis is Z cross X
    vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
    vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();

    // Create the direction cosine matrix
    matrix->Identity();
    for ( unsigned int i = 0; i < 3; ++i )
    {
        matrix->SetElement( i, 0, normalizedX[i] );
        matrix->SetElement( i, 1, normalizedY[i] );
        matrix->SetElement( i, 2, normalizedZ[i] );
    }

    // Apply the transforms
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate( p0.data() );
    transform->Concatenate(matrix);
    transform->Scale(length, thickness, thickness);

    // Transform the polydata
    vtkSmartPointer<vtkTransformPolyDataFilter> transformPD = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformPD->SetTransform(transform);
    transformPD->SetInputConnection(arrowSource->GetOutputPort());

    //Create a mapper and actor for the arrow
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> actor           = vtkSmartPointer<vtkActor>::New();

    mapper->SetInputConnection(arrowSource->GetOutputPort());
    actor->SetUserMatrix(transform->GetMatrix());
    actor->SetMapper( mapper );
    actor->GetProperty()->SetColor( color(0), color(1), color(2) );

    if ( _actors.find(name) != _actors.end() )
    {
        std::cout << "[" << __func__ << "]: " << "removing " << name << std::endl;
        _renderers[0]->RemoveActor( actor );
    }
    _actors[ name ] = actor;
    _renderers[0]->AddActor( actor );
} //...addArrow

template <typename _Scalar>
inline void Visualizer<_Scalar>::makeGif(_Scalar const angleStep, int const count, std::string pattern,
                                         int const renderer, Eigen::Vector3d const* const cameraUpArg) {
    auto const cameraUp = cameraUpArg ? *cameraUpArg
                                      : getCameraViewUp(renderer).normalized();
    setCameraViewUp(cameraUp, renderer);
    char name[255];
    for (int c = 0; c < count; ++c) {
        auto const A = Eigen::Translation3d{getCameraFocalPoint(renderer)}
                       * Eigen::AngleAxisd {angleStep, cameraUp}
                       * Eigen::Translation3d{-getCameraFocalPoint(renderer)};
        Eigen::Vector3d pos = A * getCameraPosition(renderer);
        _renderers.at(renderer)->GetActiveCamera()->SetPosition(pos.data());

        spinOnce(10);
        sprintf(name, pattern.c_str(), c);
        screenshot(name, renderer);
    }
} //...makeGif

template <typename _Scalar>
inline void Visualizer<_Scalar>::screenshot(std::string const& path, int const renderer) {
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
                                                vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(_renderWindows.at(renderer));
    //windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
    windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
    windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
    windowToImageFilter->Update();

    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(path.c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
}
} //...ns vis
} //...ns Soup

#endif // SOUP_VIS_VISUALIZER_HPP
