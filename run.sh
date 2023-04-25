export LIBTORCH=/opt/homebrew/Caskroom/miniforge/base/envs/torch/lib/python3.9/site-packages/torch
export DYLD_LIBRARY_PATH=${LIBTORCH}/lib

./marshal
