
//@file  interface dispatcher generated by intergen v6
//See LICENSE file for copyright and license information

#include "physics.h"
#include "otbullet.hpp"

#include <comm/ref.h>
#include <comm/singleton.h>
#include <comm/binstring.h>
#include <type_traits>


static_assert(intergen_interface::VERSION == 6, "interface must be rebuilt with a different intergen version");

using namespace coid;

static_assert( std::is_base_of<policy_intrusive_base, physics>::value, "class 'physics' must be derived from policy_intrusive_base");
static_assert( std::is_base_of<policy_intrusive_base, physics>::value, "class 'physics' must be derived from policy_intrusive_base");

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
        if (_vtable1) return _vtable1;

        _vtable1 = new ifn_t[34];
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
        _vtable1[15] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionObject*,unsigned int,unsigned int)>(&::physics::set_collision_info));
        _vtable1[16] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionObject*,unsigned int,unsigned int,bool)>(&::physics::add_collision_object));
        _vtable1[17] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionObject*)>(&::physics::remove_collision_object));
        _vtable1[18] = reinterpret_cast<ifn_t>(static_cast<btCompoundShape*(policy_intrusive_base::*)()>(&::physics::create_compound_shape));
        _vtable1[19] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCompoundShape*,btCollisionShape*,const btTransform&)>(&::physics::add_child_shape));
        _vtable1[20] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCompoundShape*,int,const btTransform&)>(&::physics::update_child));
        _vtable1[21] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCompoundShape*)>(&::physics::recalc_compound_shape));
        _vtable1[22] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCompoundShape*&)>(&::physics::destroy_compound_shape));
        _vtable1[23] = reinterpret_cast<ifn_t>(static_cast<btCollisionShape*(policy_intrusive_base::*)(bt::EShape,const float[3])>(&::physics::create_shape));
        _vtable1[24] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionShape*,const float[3])>(&::physics::add_convex_point));
        _vtable1[25] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionShape*)>(&::physics::close_convex_shape));
        _vtable1[26] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btCollisionShape*&)>(&::physics::destroy_shape));
        _vtable1[27] = reinterpret_cast<ifn_t>(static_cast<bt::ot_world_physics_stats(policy_intrusive_base::*)()>(&::physics::get_stats));
        _vtable1[28] = reinterpret_cast<ifn_t>(static_cast<bt::ot_world_physics_stats*(policy_intrusive_base::*)()>(&::physics::get_stats_ptr));
        _vtable1[29] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(btIDebugDraw*)>(&::physics::set_debug_draw_enabled));
        _vtable1[30] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(int)>(&::physics::set_debug_drawer_mode));
        _vtable1[31] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)()>(&::physics::debug_draw_world));
        _vtable1[32] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(const double3&,float,coid::dynarray<btCollisionObject*>&)>(&::physics::query_volume_sphere));
        _vtable1[33] = reinterpret_cast<ifn_t>(static_cast<void(policy_intrusive_base::*)(const double3&,const float4*,uint8,bool,coid::dynarray<btCollisionObject *>&)>(&::physics::query_volume_frustum));
        return _vtable1;
    }

    #define VT_CALL2(R,F,I) ((*reinterpret_cast<policy_intrusive_base*>(this)).*(reinterpret_cast<R(policy_intrusive_base::*)F>(_vtable1[I])))


    static ifn_t* get_vtable_intercept()
    {
        if (_vtable2) return _vtable2;
        ifn_t* vtable1 = get_vtable();

        _vtable2 = new ifn_t[34];
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
        _vtable2[26] = vtable1[26];
        _vtable2[27] = vtable1[27];
        _vtable2[28] = vtable1[28];
        _vtable2[29] = vtable1[29];
        _vtable2[30] = vtable1[30];
        _vtable2[31] = vtable1[31];
        _vtable2[32] = vtable1[32];
        _vtable2[33] = vtable1[33];
        return _vtable2;
    }

protected:

    COIDNEWDELETE("bt::physics_dispatcher");

    physics_dispatcher()
    {}

    bool intergen_bind_capture( coid::binstring* capture, uint instid ) override
    {
        if (instid >= 0xffU)
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

    ///Cleanup routine called from ~physics()
    static void _cleaner_callback( physics* m, intergen_interface* ifc ) {
         ::physics* host = m->host<::physics>();
        if (host) host->_ifc_host = ifc;
    }

    static iref<physics> _generic_interface_creator( ::physics* host, physics* __here__)
    {
        //cast to dispatch to sidestep protected access restrictions
        physics_dispatcher* __disp__ = static_cast<physics_dispatcher*>(__here__);
        if (!__disp__)
            __disp__ = new physics_dispatcher;

        __disp__->_host.create(host);
        __disp__->_vtable = _capture ? get_vtable_intercept() : get_vtable();
        if (!host->_ifc_host) {
            __disp__->_cleaner = &_cleaner_callback;
            host->_ifc_host = __disp__;
        }

        return __disp__;
    }

public:

    // creator methods

    static iref<physics> create( physics* __here__, double r, void* context )
    {
        iref< ::physics> __host__ = ::physics::create(r, context);
        if (!__host__)
            return 0;
        return _generic_interface_creator(__host__.get(), __here__);
    }

    static iref<physics> get( physics* __here__ )
    {
        iref< ::physics> __host__ = ::physics::get();
        if (!__host__)
            return 0;
        return _generic_interface_creator(__host__.get(), __here__);
    }

    ///Register interface creators in the global registry
    static void register_interfaces( bool on )
    {
        interface_register::register_interface_creator(
            "bt::physics@wrapper",
            on ? (void*)&_generic_interface_creator : nullptr);

        interface_register::register_interface_creator(
            "bt::physics.create@3023259009",
            on ? (void*)&create : nullptr);
        interface_register::register_interface_creator(
            "bt::physics.get@3023259009",
            on ? (void*)&get : nullptr);
    }
};

coid::binstring* physics_dispatcher::_capture = 0;
uint16 physics_dispatcher::_instid = 0xffffU;
intergen_interface::ifn_t* physics_dispatcher::_vtable2 = 0;
intergen_interface::ifn_t* physics_dispatcher::_vtable1 = 0;


//auto-register the available interface creators
LOCAL_SINGLETON_DEF(ifc_autoregger) physics_autoregger = new ifc_autoregger(&physics_dispatcher::register_interfaces);

void* force_register_physics() {
    LOCAL_SINGLETON_DEF(ifc_autoregger) autoregger = new ifc_autoregger(&physics_dispatcher::register_interfaces);
    return autoregger.get();
}

} //namespace bt

// events


bool physics::terrain_collisions( const void* context, const double3& center, float radius, float lod_dimension, coid::dynarray<bt::triangle>& data, coid::dynarray<uint>& trees, coid::slotalloc<bt::tree_batch>& tree_batches, uint frame )
{
	if (!_ifc_host) 
        throw coid::exception() << "terrain_collisions" << " handler not implemented";
    else
        return _ifc_host->iface<bt::physics>()->terrain_collisions(context, center, radius, lod_dimension, data, trees, tree_batches, frame);
}

bool physics::terrain_collisions_aabb( const void* context, const double3& center, float3x3 basis, float lod_dimension, coid::dynarray<bt::triangle>& data, coid::dynarray<uint>& trees, coid::slotalloc<bt::tree_batch>& tree_batches, uint frame, bool& is_above_tm )
{
	if (!_ifc_host) 
        throw coid::exception() << "terrain_collisions_aabb" << " handler not implemented";
    else
        return _ifc_host->iface<bt::physics>()->terrain_collisions_aabb(context, center, basis, lod_dimension, data, trees, tree_batches, frame, is_above_tm);
}

float3 physics::tree_collisions( btRigidBody* obj, bt::tree_collision_contex& ctx, float time_step, coid::slotalloc<bt::tree_batch>& tree_batches )
{
	if (!_ifc_host) 
        throw coid::exception() << "tree_collisions" << " handler not implemented";
    else
        return _ifc_host->iface<bt::physics>()->tree_collisions(obj, ctx, time_step, tree_batches);
}

float physics::terrain_ray_intersect( const void* context, const double3& from, const float3& dir, const float2& minmaxlen, float3* norm, double3* pos )
{
	if (!_ifc_host) 
        throw coid::exception() << "terrain_ray_intersect" << " handler not implemented";
    else
        return _ifc_host->iface<bt::physics>()->terrain_ray_intersect(context, from, dir, minmaxlen, norm, pos);
}

float physics::elevation_above_terrain( const double3& pos, float maxlen, float3* norm, double3* hitpoint )
{
	if (!_ifc_host) 
        throw coid::exception() << "elevation_above_terrain" << " handler not implemented";
    else
        return _ifc_host->iface<bt::physics>()->elevation_above_terrain(pos, maxlen, norm, hitpoint);
}

void physics::log( const coid::token& text )
{
	if (!_ifc_host) 
        throw coid::exception() << "log" << " handler not implemented";
    else
        return _ifc_host->iface<bt::physics>()->log(text);
}


