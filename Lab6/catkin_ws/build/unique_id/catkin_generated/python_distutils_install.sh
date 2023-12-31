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

echo_and_run cd "/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/unique_identifier/unique_id"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/install/lib/python3/dist-packages:/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/unique_id/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/unique_id" \
    "/usr/bin/python3" \
    "/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/unique_identifier/unique_id/setup.py" \
    egg_info --egg-base /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/unique_id \
    build --build-base "/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/unique_id" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/install" --install-scripts="/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/install/bin"
