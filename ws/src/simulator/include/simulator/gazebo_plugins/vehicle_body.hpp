#pragma once

#include <gazebo/common/Plugin.hh>

#include <memory>


namespace gazebo_plugins
{
class VehicleBodyPrivate;

class VehicleBody : public gazebo::ModelPlugin
{
public:
    VehicleBody();

    ~VehicleBody();
protected:
    void Load(gazebo::physics::ModelPtr model_p, sdf::ElementPtr sdf_p) override;
    
    void Reset() override;
    
private:
    std::unique_ptr<VehicleBodyPrivate> impl_p;
};
}