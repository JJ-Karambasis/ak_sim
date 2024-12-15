test_path=$(dirname "$(realpath "$0")")
dependencies_path="$test_path/dependencies"
bin_path="$test_path/bin"

if [ ! -d $bin_path ]; then
    mkdir -p $bin_path
fi

#todo: remove no-unused-function, 
warnings="-Wall -Werror -Wno-unused-function"
flags="-g -O0"

pushd $bin_path
    clang $flags $warnings -I$dependencies_path/raylib-quickstart/build/external/raylib-master/src -framework AppKit -framework IOKit $test_path/ak_sim_scene_test.c -l raylib -L $dependencies_path/raylib-quickstart/bin/Debug -o ak_sim_scene_test
    clang $flags $warnings -std=c89 -fPIC $test_path/ak_sim_compile_test.c -o ak_sim_compile_test
popd