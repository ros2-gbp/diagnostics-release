%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/iron/.*$
%global __requires_exclude_from ^/opt/ros/iron/.*$

Name:           ros-iron-diagnostic-updater
Version:        4.1.0
Release:        2%{?dist}%{?release_suffix}
Summary:        ROS diagnostic_updater package

License:        BSD-3-Clause
URL:            http://www.ros.org/wiki/diagnostic_updater
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-iron-diagnostic-msgs
Requires:       ros-iron-rclcpp
Requires:       ros-iron-rclpy
Requires:       ros-iron-std-msgs
Requires:       ros-iron-ros-workspace
BuildRequires:  ros-iron-ament-cmake
BuildRequires:  ros-iron-ament-cmake-python
BuildRequires:  ros-iron-diagnostic-msgs
BuildRequires:  ros-iron-rclcpp
BuildRequires:  ros-iron-rclpy
BuildRequires:  ros-iron-std-msgs
BuildRequires:  ros-iron-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  python%{python3_pkgversion}-pytest
BuildRequires:  ros-iron-ament-cmake-gtest
BuildRequires:  ros-iron-ament-cmake-pytest
BuildRequires:  ros-iron-ament-lint-auto
BuildRequires:  ros-iron-ament-lint-common
BuildRequires:  ros-iron-launch
BuildRequires:  ros-iron-launch-testing
BuildRequires:  ros-iron-launch-testing-ros
BuildRequires:  ros-iron-rclcpp-lifecycle
%endif

%description
diagnostic_updater contains tools for easily updating diagnostics. it is
commonly used in device drivers to keep track of the status of output topics,
device status, etc.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/iron" \
    -DAMENT_PREFIX_PATH="/opt/ros/iron" \
    -DCMAKE_PREFIX_PATH="/opt/ros/iron" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/iron

%changelog
* Thu Oct 31 2024 Austin Hendrix <namniart@gmail.com> - 4.1.0-2
- Autogenerated by Bloom

* Fri Mar 22 2024 Austin Hendrix <namniart@gmail.com> - 3.2.0-1
- Autogenerated by Bloom

* Wed May 17 2023 Austin Hendrix <namniart@gmail.com> - 3.1.2-3
- Autogenerated by Bloom

* Thu Apr 20 2023 Austin Hendrix <namniart@gmail.com> - 3.1.2-2
- Autogenerated by Bloom

* Fri Mar 24 2023 Austin Hendrix <namniart@gmail.com> - 3.1.2-1
- Autogenerated by Bloom

* Tue Mar 21 2023 Austin Hendrix <namniart@gmail.com> - 3.1.1-3
- Autogenerated by Bloom

