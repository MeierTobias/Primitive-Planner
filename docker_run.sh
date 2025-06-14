docker run -it --rm \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/ondocker":/home/hostuser/primitive-planner \
    -v "$(pwd)/src":/home/hostuser/primitive-planner/src \
    --device=/dev/dri:/dev/dri \
    --name="primitive-planner" \
    primitive-planner $@
