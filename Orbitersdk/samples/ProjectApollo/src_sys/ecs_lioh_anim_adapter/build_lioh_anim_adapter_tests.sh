#!/bin/sh
set -eu
cd "$(dirname "$0")"
ROOT=$(cd ../../../../.. && pwd)
g++ -std=c++11 -Wall -Wextra -Werror \
	-DECS_LIOH_REPO_ROOT=\"$ROOT\" \
	lioh_anim_adapter.cpp lioh_l5_review_tests.cpp \
	-o /tmp/lioh_l5_review_tests
/tmp/lioh_l5_review_tests
if grep -n ecs_lioh_anim_adapter \
	"$ROOT/Orbitersdk/samples/ProjectApollo/Build/VC2017/LEM.vcxproj" \
	"$ROOT/Orbitersdk/samples/ProjectApollo/Build/VC2017/Saturn5NASP.vcxproj" \
	"$ROOT/Orbitersdk/samples/ProjectApollo/Build/VC2017/Saturn1b.vcxproj"
then
	echo "vessel project lists the review adapter" >&2
	exit 1
fi
echo "vessel projects do not list the review adapter"
