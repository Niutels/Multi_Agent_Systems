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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hcappel1/Multi_Agent_Systems/src/spawn_robot_tools/spawn_robot_tools_pkg"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hcappel1/Multi_Agent_Systems/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hcappel1/Multi_Agent_Systems/install/lib/python2.7/dist-packages:/home/hcappel1/Multi_Agent_Systems/build/spawn_robot_tools_pkg/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hcappel1/Multi_Agent_Systems/build/spawn_robot_tools_pkg" \
    "/usr/bin/python" \
    "/home/hcappel1/Multi_Agent_Systems/src/spawn_robot_tools/spawn_robot_tools_pkg/setup.py" \
    build --build-base "/home/hcappel1/Multi_Agent_Systems/build/spawn_robot_tools_pkg" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/hcappel1/Multi_Agent_Systems/install" --install-scripts="/home/hcappel1/Multi_Agent_Systems/install/bin"
