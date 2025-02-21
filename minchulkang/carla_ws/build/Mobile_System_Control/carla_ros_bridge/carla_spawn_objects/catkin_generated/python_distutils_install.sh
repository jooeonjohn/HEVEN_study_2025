#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/minchul/carla_ws/src/Mobile_System_Control/carla_ros_bridge/carla_spawn_objects"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/minchul/carla_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/minchul/carla_ws/install/lib/python3/dist-packages:/home/minchul/carla_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/minchul/carla_ws/build" \
    "/usr/bin/python3" \
    "/home/minchul/carla_ws/src/Mobile_System_Control/carla_ros_bridge/carla_spawn_objects/setup.py" \
     \
    build --build-base "/home/minchul/carla_ws/build/Mobile_System_Control/carla_ros_bridge/carla_spawn_objects" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/minchul/carla_ws/install" --install-scripts="/home/minchul/carla_ws/install/bin"
