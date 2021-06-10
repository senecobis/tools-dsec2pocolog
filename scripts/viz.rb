#!/usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'
require 'utilrb'

include Orocos

if ARGV.size < 1 then
    puts "usage: viz.rb <data_log_directory>"
    exit
end

Orocos::CORBA::max_message_size = 100000000000
Orocos.initialize

Orocos::Process.run 'dsec2pocolog::Viz' => 'dsecviz' do

    # get the task
    STDERR.print "setting up dsecviz..."
    viz = Orocos.name_service.get 'dsecviz'
    Orocos.conf.apply(icp, ['thun_01_a_left'], :override => true )
    STDERR.puts "done"

    # logs files
    log_replay = Orocos::Log::Replay.open( ARGV[0] )

    # Log port connections
    log_replay.dsec.events.connect_to(viz.events, :type => :buffer, :size => 10)
    log_replay.dsec.frame.connect_to(viz.frame, :type => :buffer, :size => 10)

    # Configure and Run the task
    viz.configure

    viz.start

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4

    Vizkit.exec

end