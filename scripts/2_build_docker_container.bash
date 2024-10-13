#!/usr/bin/env bash

xdg-open 'http://localhost:8080/api/v3/ui/' >/dev/null 2>&1 &

docker run --rm -it \
    --network=host \
    --name swagger_server_c \
swagger_server bash -c "roslaunch nav_rest_api server.launch"
