#include "tracking/vis/visualizer.h"
#include "tracking/vis/impl/visualizer.hpp"
#include "soup/impl/pointCloudEigen.hpp"
#include "soup/pointCloudEigen.h"
#include "soup/geometryTypedefs.h"
#include "vtkObjectFactory.h"

namespace Soup {
namespace vis {
inline Eigen::Map<const Eigen::Vector3d> KeyPressInteractorStyle::getCameraPosition()
{
    return Eigen::Map<const Eigen::Vector3d>( this->GetCurrentRenderer()->GetActiveCamera()->GetPosition() );
}

inline Eigen::Map<const Eigen::Vector3d> KeyPressInteractorStyle::getCameraViewUp()
{
    return Eigen::Map<const Eigen::Vector3d>( this->GetCurrentRenderer()->GetActiveCamera()->GetViewUp() );
}

inline Eigen::Map<const Eigen::Vector3d> KeyPressInteractorStyle::getCameraFocalPoint()
{
    return Eigen::Map<const Eigen::Vector3d>( this->GetCurrentRenderer()->GetActiveCamera()->GetFocalPoint() );
}

bool KeyPressInteractorStyle::loadCamera(std::string const& path) {
    bool ret(true);
    Eigen::Vector3d position, focalPoint, viewUp;
    if ( loadCameraParams( path, position, focalPoint, viewUp) )
    {
        //std::cout << "[" << __func__ << "]: setting position: " << position.transpose() << std::endl;
        this->GetCurrentRenderer()->GetActiveCamera()->SetPosition( position.data() );
        //std::cout << "[" << __func__ << "]: setting focalPoint: " << focalPoint.transpose() << std::endl;
        this->GetCurrentRenderer()->GetActiveCamera()->SetFocalPoint( focalPoint.data() );
        //std::cout << "[" << __func__ << "]: setting viewUp: " << viewUp.transpose() << std::endl;
        this->GetCurrentRenderer()->GetActiveCamera()->SetViewUp( viewUp.data() );

        this->GetCurrentRenderer()->GetRenderWindow()->GetInteractor()->Render();

        ret = true;
    } //...if camera load successful
    else
    {
        std::cout << "[" << __func__ << "]: could not load camera.txt" << std::endl;
        ret = false;
    }

    return ret;
}

bool KeyPressInteractorStyle::loadCameraParams( const std::string& path, Eigen::Vector3d& position, Eigen::Vector3d& focalPoint, Eigen::Vector3d& viewUp )
{
    std::ifstream f( path );
    if ( !f.is_open() )
        return false;
    std::string label;

    // poisition
    f >> label;
    //std::cout << "reading " << label;
    f >> position(0) >> position(1) >> position(2);
    //std::cout << ": " << position.transpose() << std::endl;

    // focalPoint
    f >> label;
    //std::cout << "reading " << label;
    f >> focalPoint(0) >> focalPoint(1) >> focalPoint(2);
    //std::cout << ": " << focalPoint.transpose() << std::endl;

    // viewUp
    f >> label;
    //std::cout << "reading " << label;
    f >> viewUp(0) >> viewUp(1) >> viewUp(2);
    //std::cout << ": " << viewUp.transpose() << std::endl;

    f.close();
    return true;
}

void KeyPressInteractorStyle::OnKeyPress()
{
    // Get the keypress
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string                key = rwi->GetKeySym();

    // Output the key that was pressed
    //std::cout << "Pressed " << key << std::endl;

    // Handle an arrow key
    if ( key == "Up" )
    {
        std::cout << "The up arrow was pressed." << std::endl;
    }

    // Handle a "normal" key
    if ( key == "a" )
    {
        std::cout << "The a key was pressed." << std::endl;
    }
    else if ( key == "c" ) // Handle a "normal" key
    {
        std::cout << "[" << __func__ << "]: focal point: " << getCameraFocalPoint().transpose() << std::endl;
        std::cout << "[" << __func__ << "]: position: " << getCameraPosition().transpose() << std::endl;
        std::cout << "[" << __func__ << "]: position: " << getCameraViewUp().transpose() << std::endl;
    }
    else if ( key == "s" )
    {
        std::ofstream f("camera.txt");
        const Eigen::Map<const Eigen::Vector3d> pos( getCameraPosition() );
        f << "position ";
        for ( int i = 0; i != pos.rows(); ++i )
            f << pos(i) << (i == pos.rows() - 1 ? "" : " ");

        const Eigen::Map<const Eigen::Vector3d> focalPoint( getCameraFocalPoint() );
        f << "focalPoint ";
        for ( int i = 0; i != focalPoint.rows(); ++i )
            f << focalPoint(i) << (i == focalPoint.rows() - 1 ? "" : " ");

        const Eigen::Map<const Eigen::Vector3d> viewUp( getCameraViewUp() );
        f << "viewUp ";
        for ( int i = 0; i != viewUp.rows(); ++i )
            f << viewUp(i) << (i == viewUp.rows() - 1 ? "" : " ");


        f.close();
        std::cout << "saved to camera.txt" << std::endl;
    }
    else if ( key == "l" )
    {
        loadCamera("camera.txt");
    }
    else if ( key == "r" )
    {
        for ( int i = 0; i != 10; ++i )
        {

        }
    }

    // Forward events
    vtkInteractorStyleRubberBandPick::OnKeyPress();
} //...onKeyPress()
vtkStandardNewMacro(KeyPressInteractorStyle);
} //...ns vis
} //...ns Soup

// ---------------------------------------------------------------------------------------- //
//                                  Template Instantiation
// ---------------------------------------------------------------------------------------- //

namespace _visualizer_helper {
typedef Soup::geometry::HelperTypes<float> ::Vector3 Vector3f;
typedef Soup::geometry::HelperTypes<double>::Vector3 Vector3d;
}

#if 1
template class Soup::vis::Visualizer<float>;
#else
template class
Soup::vis::Visualizer<float>;
template class
Soup::vis::Visualizer<double>;

template
void Soup::vis::Visualizer<float>::addMesh( const QUASAR::Mesh &mesh, const _visualizer_helper::Vector3f* color, const std::string name, const int withNormals );
template
void Soup::vis::Visualizer<double>::addMesh( const QUASAR::Mesh &mesh, const _visualizer_helper::Vector3d* color, const std::string name, const int withNormals  );

template
void Soup::vis::Visualizer<float>::addMesh( const QUASAR::Mesh &mesh, const _visualizer_helper::Vector3f color, const std::string name, const int withNormals );
template
void Soup::vis::Visualizer<double>::addMesh( const QUASAR::Mesh &mesh, const _visualizer_helper::Vector3d color, const std::string name, const int withNormals  );
#endif