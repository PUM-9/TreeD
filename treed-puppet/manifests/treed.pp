include apt

# Hello dear future systadmin. I am deeply sorry.
# This puppet-config was written by a complete puppet-newb
# for usage on Ubuntu 14.04 LTS and ROS indigo.
#
# Please be aware that the procedure of getting ROS, 
# rospkg, python3 and ubuntu 14.04 to work together is'nt 
# pretty. ROS is installed from their PPA, however, roscore
# is installed from github with an ugly Exec (sorry), because
# apparently they conflict.

$pupfiles = "/etc/puppet/files"

node default {

apt::ppa { 
    'http://packages.ros.org/ros/ubuntu':
}

apt::key {
    'http://packages.ros.org/ros/ubuntu':
     id => 'B01FA116',
}


package {
    [
     'vim', 
     'tmux', 
     'tree',
     'python3', 
     'python3-pip',
     'python3-serial', 
     'python3-empy', 
     'python3-yaml', 
     'htop',  
     'firefox', 
     'sl',
     'git',
    ]:
      ensure => [installed, latest],
}

#package { "networkcommunication":
#    provider => dpkg,
#    ensure   => latest,
#    source   => "${pupfiles}/networkcommunication.deb",
#}

package {
     [
      'ros-indigo-desktop-full',
     ]:
      ensure => [installed, latest],
      require => Apt::Ppa['http://packages.ros.org/ros/ubuntu'];
}

# I'm sorry. Please don't find me and hit me.
exec {
    "${pupfiles}/install_rospkg.sh":
    refreshonly => true,
    subscribe => Package['ros-indigo-desktop-full'],
}

# I'm sorry. Please don't find me and hit me.
exec {
 "${pupfiles}/install_catkin_pkg.sh":
    refreshonly => true,
    subscribe => Package['ros-indigo-desktop-full'],
    require => Package['python3-pip'],
}

# I'm sorry. Please don't find me and hit me.
exec {
     "${pupfiles}/install_pip_packages.sh":
     refreshonly => true,
     subscribe => Package['python3-pip'],
}

# I'm really sorry. Please don't find me and hit me.
exec {
     "${pupfiles}/install_treed.sh":
     refreshonly => true,
     subscribe => File['/opt/treed'],
     require => Exec["${pupfiles}/install_catkin_pkg.sh"],
}

## Install various files to their places.
# I'm sorry, we did not have time to properly package things.


file { '/etc/treed/':
     ensure => directory,
     source => "$pupfiles/treeD/conf/",
     recurse => true,
     owner => 'root',
     group => 'root',
     mode => '0755',
}

file { '/opt/treed/':
     ensure => directory,
     source => "$pupfiles/treeD/",
     recurse => true,
     owner => 'root',
     group => 'root',
     mode => '0755',
}

file { '/usr/bin/treed':
     ensure => file,
     source => "$pupfiles/treeD/bin/treed",
     recurse => true,
     owner => 'root',
     group => 'root',
     mode => '0755',
}

file { '/usr/bin/treed-start-lineardrive-server':
     ensure => file,
     source => "$pupfiles/treeD/bin/treed-start-lineardrive-server",
     recurse => true,
     owner => 'root',
     group => 'root',
     mode => '0755',
}

file { '/usr/bin/treed-start-roscore':
     ensure => file,
     source => "$pupfiles/treeD/bin/treed-start-roscore",
     recurse => true,
     owner => 'root',
     group => 'root',
     mode => '0755',
}

file { '/usr/bin/treed-start-control-node':
     ensure => file,
     source => "$pupfiles/treeD/bin/treed-start-control-node",
     recurse => true,
     owner => 'root',
     group => 'root',
     mode => '0755',
}

file { '/etc/network/interfaces':
     ensure => file,
     source => "$pupfiles/interfaces",
     owner => 'root',
     group => 'root',
     mode => '0755',
}

}
