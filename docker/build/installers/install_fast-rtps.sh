#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

git clone --single-branch --branch apollo --depth 1 https://github.com/ApolloAuto/Fast-RTPS.git
pushd Fast-RTPS
git submodule init
git submodule update

mkdir -p build && cd build
cmake -DEPROSIMA_BUILD=ON -DCMAKE_INSTALL_PREFIX=/usr/local/fast-rtps ../
make -j8 fastrtps
make install
popd

rm -fr Fast-RTPS
