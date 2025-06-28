docker run -it --rm \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)":/home/hostuser/primitive-planner \
    --name="primitive-planner" \
    primitive-planner