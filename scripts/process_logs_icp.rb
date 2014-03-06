#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'transformer/runtime'
require 'vizkit'
require 'utilrb'

include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs_icp.rb <data_log_directory>"
    exit
end

#Initializes the CORBA communication layer
Orocos::CORBA::max_message_size = 100000000
Orocos.initialize

Orocos.run 'icp::GIcp' => 'icp', :gdb => false do

    # log all the output ports
    #Orocos.log_all_ports
    Orocos.conf.load_dir('../config/')

    # get the task
    STDERR.print "setting up icp..."
    icp = Orocos.name_service.get 'icp'
    Orocos.conf.apply(icp, ['default'], :override => true )
    STDERR.puts "done"

    # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] )
    log_replay.use_sample_time = true

    # Maps the logs into the input ports
    log_replay.tof.pointcloud_samples_out.connect_to(icp.point_cloud_source, :type => :buffer, :size => 200)

    # Configure and Run the task
    icp.configure
    icp.start

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4

    Vizkit.exec

end
