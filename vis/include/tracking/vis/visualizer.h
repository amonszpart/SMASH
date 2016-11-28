#ifndef SOUP_VISUALIZER_H
#define SOUP_VISUALIZER_H

#include "tracking/common/eigen.h"
#include "tracking/common/util/exception.h"
#include "soup/cloudTypedefs.h"
#include "soup/pointCloudEigen.h"
#include "soup/geometryTypedefs.h"
#include <vtkCommand.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <string>
#include <vector>
#include <map>

namespace Soup {
namespace vis {
// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleRubberBandPick {
public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleRubberBandPick);

    Eigen::Map<const Eigen::Vector3d> getCameraFocalPoint();
    Eigen::Map<const Eigen::Vector3d> getCameraPosition();
    Eigen::Map<const Eigen::Vector3d> getCameraViewUp();
    bool loadCameraParams(const std::string& path, Eigen::Vector3d& position, Eigen::Vector3d& focalPoint, Eigen::Vector3d &viewUp );
    bool loadCamera( const std::string& path = "camera.txt" );

    virtual void OnKeyPress() override final;
    virtual void PrintHeader(std::basic_ostream<char, std::char_traits<char> >& stream, vtkIndent /*indent*/ ) override final { stream << "KeyPressInteractorStyle::PrintHeader"; }
    virtual void PrintTrailer(std::basic_ostream<char, std::char_traits<char> >& stream, vtkIndent /*indent*/) override final { stream << "KeyPressInteractorStyle::PrintTrailer"; }
    virtual void PrintSelf(std::basic_ostream<char, std::char_traits<char> >& stream, vtkIndent /*indent*/) override final { stream << "KeyPressInteractorStyle::PrintSelf"; }
}; //cls KeyPressInteractorStyle

DEFINE_EXCEPTION(Visualizer);
template <typename _Scalar>
class Visualizer {
public:
    typedef _Scalar                                  Scalar;
    typedef typename geometry::ColoredCloudT::CloudT CloudT;
    typedef typename CloudT::Vector3                 Vector3;
    typedef std::shared_ptr< Visualizer<Scalar> >    PtrT;

    Visualizer( std::string name = "Visualizer" ) : _name(name) { init(); }
    void init();

    void addPointCloudMono( const CloudT                     & vertices
        , const Vector3                   & colours
        , const std::string                 name      = "cloud"
        , const Visualizer<_Scalar>::CloudT* normals   = nullptr
    );
    template <typename _Derived, typename _DerivedB>
    void addPointCloud( const Eigen::MatrixBase<_Derived > &vertices
        , const Eigen::MatrixBase<_DerivedB> &colours = CloudT()
        , const std::string                   name    = "cloud"
        , const Visualizer<_Scalar>::CloudT   *normals = nullptr
    );
    void addPointCloud( const ColoredPointCloud<_Scalar>& cloud, const std::string& name );
    template <typename _MeshT>
    void addMesh( const _MeshT &mesh, const Vector3* color, const std::string name, const int withNormals = 0 );
    template <typename _MeshT>
    void addMesh( const _MeshT &mesh, const Vector3 color, const std::string name, const int withNormals = 0 );

    void spin();
    void spinOnce( int time );

    void addSphere( const Vector3& center, Scalar radius = 1., const Vector3& colour = Vector3::Ones(), const std::string name = "sphere" );
    void addCylinder( const Vector3& center
        , const Vector3& dir
        , const Scalar radius = 1.
        , const Vector3& colour = Vector3(1.,1.,1.)
        , const std::string name = "cylinder" );

    void addCoordinateSystem( _Scalar scale = 1., const std::string name = "coordinateSystem" );
    void addLine( const Vector3 &p0, const Vector3 &p1, const Vector3& colour = Vector3::Ones(), const std::string name = "line", const Scalar lineWidth = Scalar(5.), const Scalar opacity = Scalar(0.5) );
    //void addLine( const VecType &p0, const VecType &p1, const Vector3& colour = Vector3::Ones(), const std::string name = "line", const Scalar lineWidth = Scalar(5.), const Scalar opacity = Scalar(0.5) );
    void addArrow( const Vector3 &p0, const Vector3 &p1, const Vector3& colour = Vector3::Ones(), const std::string name = "arrow", Scalar thickness = .15 );

    void addCube( const Vector3 &pos = Vector3::Zero(), const Vector3& size = Vector3::Ones(), const Vector3& color = Vector3::Ones(), const std::string& name = "cube" );

    inline void setBackgroundColor( double r, double g, double b, int rendererId = 0 )
    {
        _renderers[ rendererId ]->SetBackground( r, g, b ); // Background color
    }

    void setPointSize( double size, std::string actorName );
    void reRender();
    bool removeActor( const std::string& name );
    bool removeActorsByPrefix( const std::string& prefix );
    bool removeActorsByPrefix(std::vector<std::string> const& prefixes);
    bool removeActorsAll();
    bool hasActor(std::string const& name) const;

    void setCameraFocalPoint(Eigen::Vector3d const& focal, int const renderer = 0);
    void setCameraViewUp(Eigen::Vector3d const& up, int const renderer = 0);
    Eigen::Map<const Eigen::Vector3d> getCameraViewUp(int const renderer = 0);
    Eigen::Map<const Eigen::Vector3d> getCameraPosition(int const renderer = 0);
    Eigen::Map<const Eigen::Vector3d> getCameraFocalPoint(int const renderer = 0);
    void makeGif(_Scalar const angleStep, int const count, std::string pattern, int const renderer = 0,
                 Eigen::Vector3d const* const cameraUpArg = nullptr);
    void screenshot(std::string const& path, int const renderer = 0);
protected:
    std::string                                               _name;
    std::vector< vtkSmartPointer<vtkRenderWindowInteractor> > _interactors;
    std::vector< vtkSmartPointer<vtkRenderer              > > _renderers;
    std::vector< vtkSmartPointer<vtkRenderWindow          > > _renderWindows;
    std::map   < std::string, vtkSmartPointer<vtkActor>     > _actors;

    struct ExitMainLoopTimerCallback : public vtkCommand
    {
        static  ExitMainLoopTimerCallback* New     () { return (new ExitMainLoopTimerCallback); }
        virtual void                       Execute (vtkObject*, unsigned long event_id, void*);
        Visualizer<_Scalar> *visualizer;
        int right_timer_id;
        vtkSmartPointer<vtkRenderWindowInteractor> _interactor;
    };
    vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
}; //...Visualizer
} //...ns vis
} //...ns Soup

//namespace tracking {
//  namespace vis {
//    template <class _Scalar>
//    using Visualizer = Soup::vis::Visualizer<_Scalar>;
//  }
//}

#endif // SOUP_VISUALIZER_H

