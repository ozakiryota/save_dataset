#!/bin/bash

image_name="save_dataset"

docker build . \
	-t $image_name:latest \
	--build-arg CACHEBUST=$(date +%s)
