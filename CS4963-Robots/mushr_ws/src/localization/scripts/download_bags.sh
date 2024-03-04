#!/usr/bin/env sh

mkdir -p $(rospack find localization)
wget https://courses.cs.washington.edu/courses/cse478/data/localization_bags.tar.xz --directory-prefix $(rospack find localization)/bags/ && \
tar -C $(rospack find localization) -xvf $(rospack find localization)/bags/localization_bags.tar.xz && \
rm $(rospack find localization)/bags/localization_bags.tar.xz
