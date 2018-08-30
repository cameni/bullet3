#pragma once

#ifndef __INTERGEN_GENERATED__physics_JS_H__
#define __INTERGEN_GENERATED__physics_JS_H__

//@file Javascript interface file for physics interface generated by intergen
//See LICENSE file for copyright and license information

#include "physics.h"

#include <comm/intergen/ifc.js.h>
#include <comm/token.h>

namespace bt {
namespace js {

class physics
{
public:

    //@param scriptpath path to js script to bind to
    static iref<bt::physics> create( const ::js::script_handle& script, double r, void* context, const coid::token& bindvar = coid::token(), v8::Handle<v8::Context>* ctx=0 )
    {
        typedef iref<bt::physics> (*fn_bind)(const ::js::script_handle&, double, void*, const coid::token&, v8::Handle<v8::Context>*);
        static fn_bind binder = 0;
        static const coid::token ifckey = "bt::physics.create@creator.js";

        if (!binder)
            binder = reinterpret_cast<fn_bind>(
                coid::interface_register::get_interface_creator(ifckey));

        if (!binder)
            throw coid::exception("interface binder inaccessible: ") << ifckey;

        return binder(script, r, context, bindvar, ctx);
    }

    //@param scriptpath path to js script to bind to
    static iref<bt::physics> get( const ::js::script_handle& script, const coid::token& bindvar = coid::token(), v8::Handle<v8::Context>* ctx=0 )
    {
        typedef iref<bt::physics> (*fn_bind)(const ::js::script_handle&, const coid::token&, v8::Handle<v8::Context>*);
        static fn_bind binder = 0;
        static const coid::token ifckey = "bt::physics.get@creator.js";

        if (!binder)
            binder = reinterpret_cast<fn_bind>(
                coid::interface_register::get_interface_creator(ifckey));

        if (!binder)
            throw coid::exception("interface binder inaccessible: ") << ifckey;

        return binder(script, bindvar, ctx);
    }
};

} //namespace js
} //namespace


#endif //__INTERGEN_GENERATED__physics_JS_H__
