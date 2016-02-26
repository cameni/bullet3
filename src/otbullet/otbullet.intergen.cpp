
//@file  interface dispatcher generated by intergen v4

#include "physics.h"
#include "otbullet.hpp"

#include <comm/ref.h>
#include <comm/singleton.h>
#include <comm/binstring.h>
#include <type_traits>

using namespace coid;

static_assert( std::is_base_of<policy_intrusive_base, physics>::value, "class 'physics' must be derived from coid::policy_intrusive_base");

////////////////////////////////////////////////////////////////////////////////
// interface physics of class physics

namespace bt {

///
class physics_dispatcher : public physics
{
private:

    static coid::binstring* _capture;
    static uint16 _instid;
    static ifn_t* _vtable1;
    static ifn_t* _vtable2;

    static ifn_t* get_vtable()
    {
        if(_vtable1) return _vtable1;

        _vtable1 = new ifn_t[26];
        _vtable1[0] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(double)>(&::physics::step_simulation));
        _vtable1[1] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(const double[3],const double[3],void*)>(&::physics::ray_test));
        _vtable1[2] = reinterpret_cast<ifn_t>(static_cast<btRigidBody*(policy_intrusive_base::*)()>(&::physics::fixed_object));
        _vtable1[3] = reinterpret_cast<ifn_t>(static_cast<btRigidBody*(policy_intrusive_base::*)(float,btCollisionShape*,void*,void*)>(&::physics::create_rigid_body));
        _vtable1[4] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btRigidBody*&)>(&::physics::destroy_rigid_body));
        _vtable1[5] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btRigidBody*,unsigned int,unsigned int,btActionInterface*,bt::constraint_info*)>(&::physics::add_rigid_body));
        _vtable1[6] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btRigidBody*,btActionInterface*,bt::constraint_info*)>(&::physics::remove_rigid_body));
        _vtable1[7] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btRigidBody*,bool)>(&::physics::pause_rigid_body));
        _vtable1[8] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btRigidBody*,float,const float[3])>(&::physics::set_rigid_body_mass));
        _vtable1[9] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btRigidBody*,const double[3])>(&::physics::set_rigid_body_gravity));
        _vtable1[10] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btRigidBody*,const btTransform&,const double[3])>(&::physics::set_rigid_body_transform));
        _vtable1[11] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btRigidBody*,double,btTransform&)>(&::physics::predict_rigid_body_transform));
        _vtable1[12] = reinterpret_cast<ifn_t>(static_cast<btCollisionObject*(policy_intrusive_base::*)(btCollisionShape*,void*,void*)>(&::physics::create_collision_object));
        _vtable1[13] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionObject*&)>(&::physics::destroy_collision_object));
        _vtable1[14] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionObject*,const btTransform&,bool)>(&::physics::update_collision_object));
        _vtable1[15] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionObject*,unsigned int,unsigned int,bool)>(&::physics::add_collision_object));
        _vtable1[16] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionObject*)>(&::physics::remove_collision_object));
        _vtable1[17] = reinterpret_cast<ifn_t>(static_cast<btCompoundShape*(policy_intrusive_base::*)()>(&::physics::create_compound_shape));
        _vtable1[18] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCompoundShape*,btCollisionShape*,const btTransform&)>(&::physics::add_child_shape));
        _vtable1[19] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCompoundShape*,int,const btTransform&)>(&::physics::update_child));
        _vtable1[20] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCompoundShape*)>(&::physics::recalc_compound_shape));
        _vtable1[21] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCompoundShape*&)>(&::physics::destroy_compound_shape));
        _vtable1[22] = reinterpret_cast<ifn_t>(static_cast<btCollisionShape*(policy_intrusive_base::*)(bt::EShape,const float[3])>(&::physics::create_shape));
        _vtable1[23] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionShape*,const float[3])>(&::physics::add_convex_point));
        _vtable1[24] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionShape*)>(&::physics::close_convex_shape));
        _vtable1[25] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionShape*&)>(&::physics::destroy_shape));
        return _vtable1;
    }

    #define VT_CALL2(R,F,I) ((*reinterpret_cast<policy_intrusive_base*>(this)).*(reinterpret_cast<R(policy_intrusive_base::*)F>(_vtable1[I])))


    static ifn_t* get_vtable_intercept()
    {
        if(_vtable2) return _vtable2;
        ifn_t* vtable1 = get_vtable();

        _vtable2 = new ifn_t[26];
        _vtable2[0] = vtable1[0];
        _vtable2[1] = vtable1[1];
        _vtable2[2] = vtable1[2];
        _vtable2[3] = vtable1[3];
        _vtable2[4] = vtable1[4];
        _vtable2[5] = vtable1[5];
        _vtable2[6] = vtable1[6];
        _vtable2[7] = vtable1[7];
        _vtable2[8] = vtable1[8];
        _vtable2[9] = vtable1[9];
        _vtable2[10] = vtable1[10];
        _vtable2[11] = vtable1[11];
        _vtable2[12] = vtable1[12];
        _vtable2[13] = vtable1[13];
        _vtable2[14] = vtable1[14];
        _vtable2[15] = vtable1[15];
        _vtable2[16] = vtable1[16];
        _vtable2[17] = vtable1[17];
        _vtable2[18] = vtable1[18];
        _vtable2[19] = vtable1[19];
        _vtable2[20] = vtable1[20];
        _vtable2[21] = vtable1[21];
        _vtable2[22] = vtable1[22];
        _vtable2[23] = vtable1[23];
        _vtable2[24] = vtable1[24];
        _vtable2[25] = vtable1[25];
        return _vtable2;
    }

protected:

    physics_dispatcher()
    {}

    bool intergen_bind_capture( coid::binstring* capture, uint instid ) override
    {
        if(instid >= 0xffU)
            return false;

        _instid = uint16(instid << 8U);
        _capture = capture;
        _vtable = _capture ? get_vtable_intercept() : get_vtable();
        return true;
    }

    void intergen_capture_dispatch( uint mid, coid::binstring& bin ) override
    {
        switch(mid) {
        case UMAX32:
        default: throw coid::exception("unknown method id in physics capture dispatcher");
        }
    }

    static iref<physics> _generic_interface_creator( ::physics* host, physics* __here__)
    {
        //cast to dispatch to sidestep protected access restrictions
        physics_dispatcher* __disp__ = static_cast<physics_dispatcher*>(__here__);
        if(!__disp__)
            __disp__ = new physics_dispatcher;

        __disp__->_host.create(host);
        __disp__->_vtable = _capture ? get_vtable_intercept() : get_vtable();

        return __disp__;
    }

public:

    // creator methods

    static iref<physics> create( physics* __here__, double r, void* context )
    {
        iref< ::physics> __host__ = ::physics::create(r, context);
        if(!__host__)
            return 0;
        return _generic_interface_creator(__host__.get(), __here__);
    }

    static iref<physics> get( physics* __here__ )
    {
        iref< ::physics> __host__ = ::physics::get();
        if(!__host__)
            return 0;
        return _generic_interface_creator(__host__.get(), __here__);
    }

    ///Register interface creators in the global registry
    static void* register_interfaces()
    {
        interface_register::register_interface_creator(
            "bt::physics@wrapper", (void*)&_generic_interface_creator);

        interface_register::register_interface_creator(
            "bt::physics.create@1952619542", (void*)&create);
        interface_register::register_interface_creator(
            "bt::physics.get@1952619542", (void*)&get);

        return (void*)&register_interfaces;
    }
};

coid::binstring* physics_dispatcher::_capture = 0;
uint16 physics_dispatcher::_instid = 0xffffU;
intergen_interface::ifn_t* physics_dispatcher::_vtable2 = 0;
intergen_interface::ifn_t* physics_dispatcher::_vtable1 = 0;


//auto-register the available interface creators
static void* physics_autoregger = physics_dispatcher::register_interfaces();

void* force_register_physics() {
    return physics_dispatcher::register_interfaces();
}

} //namespace bt

