h.1 Cartographer_android

Create a toolchain 
===================
You have to download a recent ndk -- r15b for example

cd /path_to_your_ndk/build/tools
./make_standalone_toolchain.py --arch arm --api 23 --install-dir /tmp/toolchain --stl gnustl --force


Prepare libs
============

-> Download another android ndk r9d : 
https://github.com/android-ndk/ndk/wiki

-> download lua 5.2.4 : 
https://www.lua.org/ftp/lua-5.2.4.tar.gz
tar -xzf lua-5.2.4.tar.gz

-> Build libboost 
git clone https://github.com/moritz-wundke/Boost-for-Android.git
export NDK= /path_to_r9d_ndk
./build-android.sh $NDK

-> Build libceres
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver/
/path_to_r9d_ndk/ndk-build

-> Build liblua
git clone https://github.com/xxDroid/lua-android.git
cp -R /path_to_lua-android/jni  /path_to_lua-5.2.4
vim /path_to_lua-5.2.4/jni/lua.mk
Replace LIB_VERSION:=lua-5.2.2 with LIB_VERSION:=lua-5.2.4
	LOCAL_LDLIBS += -llog with LOCAL_STATIC_LIBRARIES += -llog
	include $(BUILD_SHARED_LIBRARY) with include $(BUILD_STATIC_LIBRARY)
cd /path_to_lua-5.2.4 && /path_to_r9d_ndk/ndk-build

-> Build libprotobuf
git clone https://github.com/julienr/protobuf-android
cd protobuf-android/example
./build.sh

Copy libs

