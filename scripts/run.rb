require 'orocos'
#require 'vizkit'
include Orocos

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos::CORBA::max_message_size = 100000000000
Orocos.initialize

Orocos::Process.run 'dsec2pocolog::Task' => 'dsec' do
    # log all the output ports
    Orocos.log_all_ports 
    Orocos.conf.load_dir('../config')

    # Get the task
    dsec = Orocos.name_service.get 'dsec'
    Orocos.conf.apply(dsec, ['TartanEvent'], :override => true)


    # Configure
    dsec.configure

    # Start
    dsec.start

    # No visualization
    #Vizkit.exec
end
