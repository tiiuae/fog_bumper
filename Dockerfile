FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-latest AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb
# 3) writes the .deb packages to build_output/
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-d2cdcdb

ENTRYPOINT /entrypoint.sh

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-fog-bumper_*_amd64.deb /fog-bumper.deb

RUN apt update && apt install -y --no-install-recommends ./fog-bumper.deb \
	&& rm /fog-bumper.deb
