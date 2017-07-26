.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

====================
Cartographer_android
====================

.. contents::

.. section-numbering::

Create a toolchain 
===================
Download a recent ndk (r15b for example)

.. code-block:: bash

   $ cd /path_to_your_ndk/build/tools
   $ ./make_standalone_toolchain.py --arch arm --api 23 --install-dir /tmp/toolchain --stl gnustl --force


Prepare libraries
=================

Download another android ndk r9d : https://github.com/android-ndk/ndk/wiki

Download lua 5.2.4 : https://www.lua.org/ftp/lua-5.2.4.tar.gz

.. code-block:: bash

   $ tar -xzf lua-5.2.4.tar.gz

Build libboost 
--------------

.. code-block:: bash

   $ git clone https://github.com/moritz-wundke/Boost-for-Android.git
   $ export NDK= /path_to_r9d_ndk
   $ ./build-android.sh $NDK

Build libceres
--------------

.. code-block:: bash

   $ git clone https://github.com/ceres-solver/ceres-solver.git
   $ cd ceres-solver/
   $ /path_to_r9d_ndk/ndk-build

Build liblua
------------

.. code-block:: bash

   $ git clone https://github.com/xxDroid/lua-android.git
   $ cp -R /path_to_lua-android/jni  /path_to_lua-5.2.4
   $ vim /path_to_lua-5.2.4/jni/lua.mk

Then replace ``LIB_VERSION:=lua-5.2.2`` with ``LIB_VERSION:=lua-5.2.4``, ``LOCAL_LDLIBS += -llog`` with ``LOCAL_STATIC_LIBRARIES += -llog`` and	``include $(BUILD_SHARED_LIBRARY)`` with ``include $(BUILD_STATIC_LIBRARY)``.

.. code-block:: bash

   $ cd /path_to_lua-5.2.4 && /path_to_r9d_ndk/ndk-build

Build libprotobuf
-----------------

.. code-block:: bash

   $ git clone https://github.com/julienr/protobuf-android
   $ cd protobuf-android/example
   $ ./build.sh

Copy libs
---------

After cloning our rep `android_cartographer`_

.. code-block:: bash
   
   $ cd /path_to_android_cartographer/.. 
   $ mkdir libs 
   $ cd libs
   $ cp /path_to_Boost-for-Android/build/lib/libboost_iostreams-gcc-mt-1_53.a .
   $ cp /path_to_ceres-solver/obj/local/armeabi-v7a/libceres.a .
   $ cp /path_to_lua-5.2.4/obj/local/armeabi-v7a/liblua.a .
   $ cp /path_to_protobuf-android/example/obj/local/armeabi-v7a/libprotobuf.a .
   

