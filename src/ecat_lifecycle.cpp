#include <ecat_lifecycle.hpp>

using namespace EthercatLifeCycleNode ; 

EthercatLifeCycle::EthercatLifeCycle()
{
    
    ecat_node_= std::make_unique<EthercatNode>();

    received_data_.status_word.resize(g_kNumberOfServoDrivers);
    received_data_.actual_pos.resize(g_kNumberOfServoDrivers);
    received_data_.actual_vel.resize(g_kNumberOfServoDrivers);
    received_data_.actual_tor.resize(g_kNumberOfServoDrivers);
    received_data_.op_mode_display.resize(g_kNumberOfServoDrivers);

    sent_data_.control_word.resize(g_kNumberOfServoDrivers);
    sent_data_.target_pos.resize(g_kNumberOfServoDrivers);
    sent_data_.target_vel.resize(g_kNumberOfServoDrivers);
    sent_data_.target_tor.resize(g_kNumberOfServoDrivers);
}

EthercatLifeCycle::~EthercatLifeCycle()
{
    ecat_node_.reset();
}

uint8_t EthercatLifeCycle::on_configure()
{
    printf( "Configuring EtherCAT device...\n");
    if(InitEthercatCommunication())
    {
        printf( "Configuration phase failed");
        return FAILURE;
    }else{

        /// Add qt signal and sloth mechanism in here.
        return SUCCESS;
    }
}

uint8_t EthercatLifeCycle::on_activate()
{
    if(StartEthercatCommunication()){

        printf("Activation phase failed");
        return FAILURE;
    }else{
        // received_data_publisher_->on_activate();
        // sent_data_publisher_->on_activate();
        printf( "Activation complete, real-time communication started.");
        return SUCCESS;
    }
}

uint8_t EthercatLifeCycle::on_deactivate()
{
    printf( "Deactivating.");
    // received_data_publisher_->on_deactivate();
    // sent_data_publisher_->on_deactivate();
    ecat_node_->DeactivateCommunication();
    return SUCCESS;
}

uint8_t EthercatLifeCycle::on_cleanup()
{
    printf( "Cleaning up.");
    ecat_node_.reset();
    // received_data_publisher_.reset();
    // sent_data_publisher_.reset();
    return SUCCESS;
}

uint8_t EthercatLifeCycle::on_shutdown()
{
    printf( "On_Shutdown... Waiting for control thread.");
    sig = 0;
    usleep(1e3);
    pthread_join(ethercat_thread_,NULL);
    printf( "Control thread terminated.");
    ecat_node_->ReleaseMaster();
    ecat_node_->ShutDownEthercatMaster();
    return SUCCESS;
}

uint8_t EthercatLifeCycle::on_error()
{
    printf( "On Error.");
    ecat_node_.reset();
    return SUCCESS;
}

int EthercatLifeCycle::SetComThreadPriorities()
{
    ethercat_sched_param_.sched_priority = 98;
    printf("Using priority %i\n.", ethercat_sched_param_.sched_priority);

    if (sched_setscheduler(0, SCHED_FIFO, &ethercat_sched_param_) == -1){
        printf( "Set scheduler failed. ! ");
        return -1 ; 
    }    
    err_ = pthread_attr_init(&ethercat_thread_attr_);
    if (err_) {
        printf( "Error initializing thread attribute  ! ");
        return -1;
    }
    /**********************************************************************************************/
    // This part is for CPU isolation to dedicate one core for EtherCAT communication.
    // for this feature to be active fist you have to modify GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub 
    // add isolcpus=3 so after editing it will be ; GRUB_CMDLINE_LINUX_DEFAULT = "quiet splash isolcpus=3" 
    // save and exit, and type sudo update-grub and reboot.
    //  cpu_set_t mask;
    // CPU_ZERO(&mask);
    // CPU_SET(3,&mask);

    // int result = sched_setaffinity(0,sizeof(mask),&mask);
    /**********************************************************************************************/
    
    /* Set a specific stack size  */
    err_ = pthread_attr_setstacksize(&ethercat_thread_attr_, 4096*64);
    if (err_) {
        printf( "Error setting thread stack size  ! ");
        return -1 ;
    }

    err_ = pthread_attr_setschedpolicy(&ethercat_thread_attr_, SCHED_FIFO);
    if (err_) {
        printf( "Pthread setschedpolicy failed ! ");
        return -1 ;
    }
    err_ = pthread_attr_setschedparam(&ethercat_thread_attr_, &ethercat_sched_param_);
    if (err_) {
            printf( "Pthread setschedparam failed ! ");
            return -1 ;
    }
    /* Use scheduling parameters of attr */
    err_ = pthread_attr_setinheritsched(&ethercat_thread_attr_, PTHREAD_EXPLICIT_SCHED);
    if (err_) 
    {
        printf( "Pthread setinheritsched failed ! ");
        return -1 ;
    }
}

int EthercatLifeCycle::InitEthercatCommunication()
{
    printf("Opening EtherCAT device...\n");
    if (ecat_node_->OpenEthercatMaster())
    {
        return -1 ;
    }

    printf("Configuring EtherCAT master...\n");
    if (ecat_node_->ConfigureMaster())
    {
        return -1 ;
    }

    printf("Getting connected slave informations...\n");
    if(ecat_node_->GetNumberOfConnectedSlaves()){
        return -1 ;
    }

    ecat_node_->GetAllSlaveInformation();
    for(int i = 0 ; i < NUM_OF_SLAVES ; i++){
        printf("--------------------Slave Info -------------------------\n"
               "Slave alias         = %d\n "
               "Slave position      = %d\n "
               "Slave vendor_id     = 0x%08x\n "
               "Slave product_code  = 0x%08x\n "
               "Slave name          = %s\n "
               "--------------------EOF %d'th Slave Info ----------------\n ",
                ecat_node_->slaves_[i].slave_info_.alias,
                ecat_node_->slaves_[i].slave_info_.position,
                ecat_node_->slaves_[i].slave_info_.vendor_id,
                ecat_node_->slaves_[i].slave_info_.product_code,
                ecat_node_->slaves_[i].slave_info_.name,i);
    }

    printf("Configuring  slaves...\n");
    if(ecat_node_->ConfigureSlaves()){
        return -1 ;
    }
#if VELOCITY_MODE
    ProfileVelocityParam P ;
    
    P.profile_acc=3e4 ;
    P.profile_dec=3e4 ;
    P.max_profile_vel = 7000 ;
    P.quick_stop_dec = 3e4 ;
    P.motion_profile_type = 0 ;
    ecat_node_->SetProfileVelocityParametersAll(P);
#endif

#if POSITION_MODE
    ProfilePosParam P ;
    uint32_t max_fol_err ;
    P.profile_vel = 450; //150 ;
    P.profile_acc = 1e4;//1e4 ;
    P.profile_dec = 1e4;//1e4 ;
    P.max_profile_vel = 500; //100 ;
    P.quick_stop_dec = 3e4;//3e4 ;
    P.motion_profile_type = 0 ;
    ecat_node_->SetProfilePositionParametersAll(P);
#endif

#if CYCLIC_POSITION_MODE
    CSPositionModeParam P ;
    uint32_t max_fol_err ;
    P.profile_vel = 50 ;
    P.profile_acc = 3e4 ;
    P.profile_dec = 3e4 ;
    P.max_profile_vel = 100 ;
    P.quick_stop_dec = 3e4 ;
    P.interpolation_time_period = 0;//1 ;
    ecat_node_->SetCyclicSyncPositionModeParametersAll(P);
#endif

#if CYCLIC_VELOCITY_MODE
    printf("Setting drives to CSV mode...\n");
    CSVelocityModeParam P ;
    P.velocity_controller_gain.Pgain = 40000;
    P.velocity_controller_gain.Igain = 800000;
    P.profile_dec=3e4 ;
    P.quick_stop_dec = 3e4 ;
    P.interpolation_time_period = 0;//1 ;
    ecat_node_->SetCyclicSyncVelocityModeParametersAll(P);
#endif

    printf("Mapping default PDOs...\n");
    if(ecat_node_->MapDefaultPdos()){
        return  -1 ;
    }

    printf("Configuring DC synchronization...\n");
    ecat_node_->ConfigDcSyncDefault();

    printf("Activating master...\n");
    if(ecat_node_->ActivateMaster()){
        return  -1 ;
    }

    printf("Registering master domain...\n");
    if (ecat_node_->RegisterDomain()){
        return  -1 ;
    }

    if (ecat_node_->WaitForOperationalMode()){
        return -1 ;
    }

    if (SetComThreadPriorities()){
        return -1 ;
    }
    printf("Initialization succesfull...\n");
    
    return 0 ; 
}

int  EthercatLifeCycle::StartEthercatCommunication()
{
    err_= pthread_create(&ethercat_thread_,&ethercat_thread_attr_, &EthercatLifeCycle::PassCycylicExchange,this);
    if(err_)
    {
        printf( "Error : Couldn't start communication thread.!");
        return -1 ; 
    }
    printf( "Communication thread called.\n");
    return 0 ;
}

void *EthercatLifeCycle::PassCycylicExchange(void *arg)
{
    static_cast<EthercatLifeCycle*>(arg)->StartPdoExchange(arg);
}

void EthercatLifeCycle::StartPdoExchange(void *instance)
{
    printf( "Starting PDO exchange....\n");
    // Measurement time in minutes, e.g.
    uint32_t print_max_min = measurement_time * 60000 ; 
    uint32_t print_val = 1e4;
    int error_check=0;
    struct timespec wake_up_time, time, publish_time_start={}, publish_time_end={};
    #if MEASURE_TIMING
        struct timespec start_time, end_time, last_start_time = {};
        uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
        latency_min_ns = 0xffffffff, latency_max_ns = 0,
        period_min_ns = 0xffffffff, period_max_ns = 0,
        exec_min_ns = 0xffffffff, exec_max_ns = 0,
        max_period=0, max_latency=0,exec_max=0,min_period = 0xffffffff,
        exec_min = 0xffffffff , latency_min = 0xffffffff;
        int32_t publishing_time_ns=1e4, publish_time_max=0, publish_time_min=0xfffffff;
        int32_t jitter = 0 , jitter_min = 0xfffffff, jitter_max = 0, old_latency=0;

    #endif
    // get current time
    clock_gettime(CLOCK_TO_USE, &wake_up_time);
    int begin=1e4;
    int status_check_counter = 1000;
    
    // ------------------------------------------------------- //
    // CKim - Initialization loop before entring control loop. 
    // Switch On and Enable Driver
    printf( "Enabling motors...");
    while(sig)
    {
        // CKim - Sleep for 1 ms
        wake_up_time = timespec_add(wake_up_time, g_cycle_time);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
        ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));

        // CKim - Receive process data
        ecrt_master_receive(g_master);
        ecrt_domain_process(g_master_domain);
        ReadFromSlaves();

        // CKim - Initialize target pos and vel
        for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++)
        {
            sent_data_.target_pos[i] = received_data_.actual_pos[i];
            sent_data_.target_vel[i] = 0;
        }

        // CKim - Check status and update control words to enable drivers
        // Returns number of enabled drivers
        if(EnableDrivers()==g_kNumberOfServoDrivers)
        
        {
            printf( "All drives enabled");
            break;
        }

        // CKim - Periodic printout
        if (status_check_counter){
            status_check_counter--;
        }
        else 
        { 
            // Checking master/domain/slaves state every 1sec.
            if(ecat_node_->CheckMasterState() < 0 )
            {
                printf( "Connection error, check your physical connection.");
                al_state_ = g_master_state.al_states ; 
                received_data_.emergency_switch_val=0;
                emergency_status_=0;
                PublishAllData();
                error_check++;                    
                if(error_check==5)
                    return;
            }
            else
            {
                //ecat_node_->CheckMasterDomainState();
                //ecat_node_->CheckSlaveConfigurationState();
                error_check=0;
                al_state_ = g_master_state.al_states ; 
                status_check_counter = 1000;

                for(int i=0; i<g_kNumberOfServoDrivers; i++)
                {
                    printf( "State of Drive %d : %d\n",i,motor_state_[i]);
                    printf( "Trying to enable motors");
                } 
            }
        }

        // CKim - Queue data
        PublishAllData();
        
        //WriteToSlavesInPositionMode();
        WriteToSlavesVelocityMode();
        ecrt_domain_queue(g_master_domain);
        // CKim - Sync Timer
        clock_gettime(CLOCK_TO_USE, &time);
        ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(time));
        ecrt_master_sync_slave_clocks(g_master);

        // CKim - Send process data
        ecrt_master_send(g_master);
    }// while(sig)
    printf( "All motors enabled, entering control loop");

    // ------------------------------------------------------- //
    // CKim - All motors enabled. Start control loop
    while(sig){
        wake_up_time = timespec_add(wake_up_time, g_cycle_time);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
        ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));
        
        #if MEASURE_TIMING
            clock_gettime(CLOCK_TO_USE, &start_time);
            old_latency = latency_ns;
            latency_ns = DIFF_NS(wake_up_time, start_time);
            period_ns = DIFF_NS(last_start_time, start_time);
            exec_ns = DIFF_NS(last_start_time, end_time);
            last_start_time = start_time;
            if(!begin)
            {
                jitter = latency_ns - old_latency ;
                if(jitter < 0 )                         jitter *=-1; 
                if(jitter > jitter_max)             jitter_max  = jitter ; 
                if(latency_ns > max_latency)        max_latency = latency_ns;
                if(period_ns > max_period)          max_period  = period_ns;
                if(exec_ns > exec_max)              exec_max    = exec_ns;
                if(period_ns < min_period)          min_period  = period_ns;
                if(exec_ns < exec_min)              exec_min    = exec_ns;
                if(latency_ns < latency_min)        latency_min = latency_ns;
                if(jitter < jitter_min)             jitter_min  = jitter;
            }

            if (latency_ns > latency_max_ns)  {
                latency_max_ns = latency_ns;
            }
            if (latency_ns < latency_min_ns) {
                latency_min_ns = latency_ns;
            }
            if (period_ns > period_max_ns) {
                period_max_ns = period_ns;
            }
            if (period_ns < period_min_ns) {
                period_min_ns = period_ns;
            }
            if (exec_ns > exec_max_ns) {
                exec_max_ns = exec_ns;
            }
            if (exec_ns < exec_min_ns) {
                exec_min_ns = exec_ns;
            }
        #endif

        // receive process data
        ecrt_master_receive(g_master);
        ecrt_domain_process(g_master_domain);

        if (status_check_counter){
            status_check_counter--;
        }
        else { 
            // Checking master/domain/slaves state every 1sec.
               if(ecat_node_->CheckMasterState() < 0 ){
                    printf( "Connection error, check your physical connection.");
                    al_state_ = g_master_state.al_states ; 
                    received_data_.emergency_switch_val=0;
                    emergency_status_=0;
                    PublishAllData();
                    error_check++;                    
                    if(error_check==5)
                        return;
                    }else{
                        // ecat_node_->CheckMasterDomainState();
                        // ecat_node_->CheckSlaveConfigurationState();
                        error_check=0;
                        al_state_ = g_master_state.al_states ; 
                        status_check_counter = 1000;
                    }
            }

        #if MEASURE_TIMING
        if(!begin)
        clock_gettime(CLOCK_TO_USE, &publish_time_start);
        #endif
        
        // timer_info_.GetTime();
        PublishAllData();
        // timer_info_.MeasureTimeDifference();
        // if(timer_info_.counter_==NUMBER_OF_SAMPLES){
        //     timer_info_.OutInfoToFile();
        //     //break;
        // }
        #if MEASURE_TIMING
        if(!begin)
        clock_gettime(CLOCK_TO_USE, &publish_time_end);
        publishing_time_ns = DIFF_NS(publish_time_start,publish_time_end);
        if(publishing_time_ns>publish_time_max) publish_time_max = publishing_time_ns;
        if(publishing_time_ns<publish_time_min) publish_time_min = publishing_time_ns;
        #endif

        #if MEASURE_TIMING
            // output timing stats
            if(!print_val){
                    // printf("-----------------------------------------------\n\n");
                    // printf("Tperiod   min   : %10u ns  | max : %10u ns\n",
                    //         period_min_ns, period_max_ns);
                    // printf("Texec     min   : %10u ns  | max : %10u ns\n",
                    //         exec_min_ns, exec_max_ns);
                    // printf("Tlatency  min   : %10u ns  | max : %10u ns\n",
                    //         latency_min_ns, latency_max_ns);
                    // printf("Tjitter max     : %10u ns  \n",
                    //         latency_max_ns-latency_min_ns);
                    // printf("-----------------------------------------------\n\n");       
                    // printf("Tperiod min     : %10u ns  | max : %10u ns\n",
                    //         min_period, max_period);
                    // printf("Texec  min      : %10u ns  | max : %10u ns\n",
                    //         exec_min, exec_max);
                    // printf("Tjitter min     : %10u ns  | max : %10u ns\n",
                    //         jitter_min, jitter_max);  
                    // printf("Publish time min: %10d ns  | max : %10d ns\n",
                    //       publish_time_min, publish_time_max);                             
                    // printf("-----------------------------------------------\n\n");
                    // std::cout << min_period << " " << max_period << " " << exec_min << " " << exec_max << " " << jitter_min << " " << jitter_max << std::endl;
                    /*  std::cout <<    "Left Switch   : " << unsigned(received_data_.left_limit_switch_val) << std::endl << 
                                        "Right Switch  : " << unsigned(received_data_.right_limit_switch_val) << std::endl;
                        std::cout << "Left X Axis    : " << controller_.left_x_axis_ << std::endl;
                        std::cout << "Right X XAxis  : " << controller_.right_x_axis_ << std::endl;*/
                        // std::cout << "Emergency button  : " << unsigned(gui_node_data_) << std::endl;
                    //std::cout << std::dec << publishing_time_ns << std::endl;
                    //std::cout << std::dec << time_span.count() << std::endl;
                    print_val=10;

                    //  std::cout << "Finished...." << std::endl;
                    //  break;
            }else {
                //std::cout << std::dec << period_min_ns  << " " << period_max_ns << " " << exec_min_ns << " " << exec_max_ns << " " << 
               // latency_min_ns << " " <<  latency_max_ns << " " << latency_max_ns - latency_min_ns << std::endl;
                print_val--;
            }
            if(!print_max_min){
                //printf("Publish time min: %10d ns  | max : %10d ns\n",
                //publish_time_min, publish_time_max);
                break;
            }
                print_max_min--;        
                period_max_ns = 0;
                period_min_ns = 0xffffffff;

                exec_max_ns = 0;
                exec_min_ns = 0xffffffff;

                latency_max_ns = 0;
                latency_min_ns = 0xffffffff;
        #endif

        ReadFromSlaves();
#if POSITION_MODE
        UpdateMotorStatePositionMode();
        UpdatePositionModeParameters();
        WriteToSlavesInPositionMode();
#endif
#if CYCLIC_POSITION_MODE
    UpdateMotorStatePositionMode();
    UpdateCyclicPositionModeParameters();
    //WriteToSlavesInPositionMode();
#endif 
#if VELOCITY_MODE
        UpdateMotorStateVelocityMode();
        UpdateVelocityModeParameters();
        WriteToSlavesVelocityMode();
#endif
#if CYCLIC_VELOCITY_MODE
        UpdateMotorStateVelocityMode();
        UpdateCyclicVelocityModeParameters();
        WriteToSlavesVelocityMode();
#endif
        ecrt_domain_queue(g_master_domain);
        clock_gettime(CLOCK_TO_USE, &time);
        ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(time));
        ecrt_master_sync_slave_clocks(g_master);
        // send process data
        ecrt_master_send(g_master);
        
        if(begin) begin--;
        #if MEASURE_TIMING
                clock_gettime(CLOCK_TO_USE, &end_time);
        #endif
    }//while(1/sig) //Ctrl+C signal
    
    // ------------------------------------------------------- //
    // CKim - Disable drivers before exiting
    wake_up_time = timespec_add(wake_up_time, g_cycle_time);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
    ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));

    ecrt_master_receive(g_master);
    ecrt_domain_process(g_master_domain);

    ReadFromSlaves();
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++)
    {
        sent_data_.control_word[i] = SM_GO_SWITCH_ON_DISABLE;
    }
    WriteToSlavesVelocityMode();

    ecrt_domain_queue(g_master_domain);
    ecrt_master_send(g_master);
    usleep(10000);
    // ------------------------------------------------------- //

    printf( "Leaving control thread.");
    ecat_node_->DeactivateCommunication();
    return;
}// StartPdoExchange end

void EthercatLifeCycle::ReadFromSlaves()
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        received_data_.actual_pos[i]  = EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_pos);
        received_data_.actual_vel[i]  = EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_vel);
        received_data_.status_word[i] = EC_READ_U16(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.status_word);
    }
    received_data_.com_status = al_state_ ; 
    received_data_.right_limit_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.r_limit_switch);
    received_data_.left_limit_switch_val  = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.l_limit_switch);
    received_data_.emergency_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.emergency_switch);
    emergency_status_  = received_data_.emergency_switch_val;
}// ReadFromSlaves end

void EthercatLifeCycle::WriteToSlavesVelocityMode()
{
  //  printf( "Writing to slaves....\n");
  if(!emergency_status_ || !gui_node_data_){
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
        EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_vel,0);
    }
  }else{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
        EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_vel,sent_data_.target_vel[i]);
    }
  }
}

int EthercatLifeCycle::PublishAllData()
{   
   // printf( "Publishing all data....\n");
}

int EthercatLifeCycle::GetComState()
{
    return al_state_ ; 
}

void EthercatLifeCycle::UpdatePositionModeParameters()
{   
       // printf( "Updating control parameters....\n");
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached || motor_state_[i]==kSwitchedOn){
            if (controller_.xbox_button_){
                for(int j = 0 ; j < g_kNumberOfServoDrivers ; j++){
                    sent_data_.target_pos[j] = 0 ; 
                    sent_data_.control_word[j] = SM_GO_ENABLE ;
                }
                break;
            }
            // Settings for motor 1;
            if(controller_.red_button_ > 0 ){
                sent_data_.target_pos[0] = -FIVE_DEGREE_CCW ;
            }
            if(controller_.blue_button_ > 0){
                sent_data_.target_pos[0] = FIVE_DEGREE_CCW ;
            }
            if(controller_.green_button_ > 0 ){
                sent_data_.target_pos[0] = THIRTY_DEGREE_CCW ;
            }
            if(controller_.yellow_button_ > 0){
                sent_data_.target_pos[0] = -THIRTY_DEGREE_CCW ;
            }
            
            if(controller_.red_button_ || controller_.blue_button_ || controller_.green_button_ || controller_.yellow_button_){
                sent_data_.control_word[0] = SM_GO_ENABLE;
            }
            // Settings for motor 2 
            if(controller_.left_r_button_ > 0 ){
                sent_data_.target_pos[1] = FIVE_DEGREE_CCW ;
            }
            if(controller_.left_l_button_ > 0){
                sent_data_.target_pos[1] = -FIVE_DEGREE_CCW ;
            }
            if(controller_.left_u_button_ > 0 ){
                sent_data_.target_pos[1] = -THIRTY_DEGREE_CCW ;
            }
            if(controller_.left_d_button_ > 0){
                sent_data_.target_pos[1] = THIRTY_DEGREE_CCW ;
            }

            if((controller_.left_r_button_ || controller_.left_l_button_ || controller_.left_u_button_ || controller_.left_d_button_)){
                sent_data_.control_word[1] = SM_GO_ENABLE;
            }

            // Settings for motor 3 
            if(controller_.right_rb_button_ > 0 ){
                sent_data_.target_pos[2] = -FIVE_DEGREE_CCW ;
            }
            if(controller_.left_rb_button_ > 0){
                sent_data_.target_pos[2] = FIVE_DEGREE_CCW ;
            }
            if(controller_.left_start_button_ > 0 ){
                sent_data_.target_pos[2] = THIRTY_DEGREE_CCW ;
            }
            if(controller_.right_start_button_ > 0){
                sent_data_.target_pos[2] = -THIRTY_DEGREE_CCW ;
            }
            if((controller_.right_rb_button_ || controller_.left_rb_button_ || controller_.left_start_button_ || controller_.right_start_button_)){
                sent_data_.control_word[2] = SM_GO_ENABLE;
            }
        }
    }
}

void EthercatLifeCycle::UpdateMotorStatePositionMode()
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if ((received_data_.status_word[i] & command_) == 0X08){             
                //if status is fault, reset fault state.
                command_ = 0X04F;
                sent_data_.control_word[i] = SM_FULL_RESET;
                motor_state_[i] = kFault;
        }
        if(motor_state_[i]!=kSwitchedOn){
            sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
            if ( (received_data_.status_word[i] & command_) == 0x0040){  
                // If status is "Switch on disabled", \
                change state to "Ready to switch on"
                sent_data_.control_word[i]  = SM_GO_READY_TO_SWITCH_ON;
                command_ = 0x006f;
                motor_state_[i] = kSwitchOnDisabled;
            } else if ( (received_data_.status_word[i] & command_) == 0x0021){
                    // If status is "Ready to switch on", \
                        change state to "Switched on"
                sent_data_.control_word[i]  = SM_GO_SWITCH_ON;     
                command_ = 0x006f;
                motor_state_[i] = kReadyToSwitchOn;

            } else if ( (received_data_.status_word[i] & command_) == 0x0023){         
                // If status is "Switched on", change state to "Operation enabled"
                sent_data_.control_word[i]  = SM_GO_ENABLE;
                command_ = 0x006f;
                motor_state_[i] = kSwitchedOn;

            }else if ((received_data_.status_word[i] & command_) == 0X08){             
                //if status is fault, reset fault state.
                command_ = 0X04f;

                sent_data_.control_word[i]  = SM_FULL_RESET;
                motor_state_[i] = kFault;
            }
        }
        else {
            sent_data_.control_word[i]=SM_RUN;
            if(TEST_BIT(received_data_.status_word[i],10)==1)
            {
                sent_data_.control_word[i]=SM_RELATIVE_POS;
            }
        }
    }
    return ;
}

int EthercatLifeCycle::GetDriveState(const int& statusWord)
{
    int state = 0;

    // bit 6 is 1
    if (TEST_BIT(statusWord,6)) 
    {   
        state = kSwitchOnDisabled;  return state;   
    }
    
    // bit 6 is 0 and bit 5 is 1
    if (TEST_BIT(statusWord,5)) 
    {
        if (TEST_BIT(statusWord,2)) {    
            state = kOperationEnabled;  return state;   
        }
        if (TEST_BIT(statusWord,1)) {    
            state = kSwitchedOn;        return state;   
        }
        if (TEST_BIT(statusWord,0)) {    
            state = kReadyToSwitchOn;   return state;
        }
    }
    
    // bit 6 is 0 and bit 5 is 0
    if (TEST_BIT(statusWord,3)) {
        // For EPOS4, Fault or Fault Reaction Active,
        // See P2-14 of the Firmware Manual
        state = kFault;         return state;   
    }
    else {
        // For EPOS4, Quick Stop Active or Not Switched on
        // See P2-14 of the Firmware Manual
        state = kQuickStop;     return state;
    }
    return state;
}

int EthercatLifeCycle::EnableDrivers()
{
    int cnt = 0;
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++)
    {
        motor_state_[i] = GetDriveState(received_data_.status_word[i]);
        
        // if status is fault, reset fault state.
        if(motor_state_[i] == kFault) {
            //printf( "Driver %d in Fault",i);
            sent_data_.control_word[i] = SM_FULL_RESET;
        }

        // If status is "Switch on disabled", change state to "Ready to switch on"
        if(motor_state_[i] == kSwitchOnDisabled) {
            sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
        }

        // If status is "Ready to switch on", change state to "Switched on"
        if(motor_state_[i] == kReadyToSwitchOn) {
            sent_data_.control_word[i]  = SM_GO_SWITCH_ON;     
        }
        
        // If status is "Switched on", change state to "Operation enabled"
        if(motor_state_[i] == kSwitchedOn) {
            sent_data_.control_word[i]  = SM_GO_ENABLE;
        }

        // If status is "Switched on", change state to "Operation enabled"
        if(motor_state_[i] == kOperationEnabled) {
            cnt++;
        }
    }        
    return cnt;
}

void EthercatLifeCycle::WriteToSlavesInPositionMode()
{
    if(!received_data_.left_limit_switch_val || !received_data_.right_limit_switch_val){
        for(int i = 0 ; i < g_kNumberOfServoDrivers; i++){
            if(sent_data_.target_pos[i] > 0){
                EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
                EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_pos,sent_data_.target_pos[i]);
            }else{
                EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,SM_QUICKSTOP);
            }
        }
    }else {
        if(!emergency_status_ || !gui_node_data_){
            for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
                EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,SM_QUICKSTOP);
        //      EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_pos,0);
            }
        }else{
            for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
                EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
                EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_pos,sent_data_.target_pos[i]);
            }
        }
    }
}

// void EthercatLifeCycle::UpdateCyclicPositionModeParameters()
// {
//     // printf( "Updating control parameters....\n");
//     for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
//         if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached || motor_state_[i]==kSwitchedOn){
//             if (controller_.xbox_button_){
//                 for(int j = 0 ; j < g_kNumberOfServoDrivers ; j++){
//                     sent_data_.target_pos[j] = 0 ; 
//                     sent_data_.control_word[j] = SM_GO_ENABLE ;
//                 }
//                 break;
//             }
//             // Settings for motor 1;
//             if(controller_.red_button_ > 0 ){
//                 sent_data_.target_pos[0] = received_data_.actual_pos[0] - FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.blue_button_ > 0){
//                 sent_data_.target_pos[0] = received_data_.actual_pos[0] + FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.green_button_ > 0 ){
//                 sent_data_.target_pos[0] = received_data_.actual_pos[0] + THIRTY_DEGREE_CCW/50 ;
//             }
//             if(controller_.yellow_button_ > 0){
//                 sent_data_.target_pos[0] = received_data_.actual_pos[0] -THIRTY_DEGREE_CCW/50 ;
//             }
//            
//             if(controller_.red_button_ || controller_.blue_button_ || controller_.green_button_ || controller_.yellow_button_){
//                 sent_data_.control_word[0] = SM_GO_ENABLE;
//             }
//             // Settings for motor 2 
//             if(controller_.left_r_button_ > 0 ){
//                 sent_data_.target_pos[1] = received_data_.actual_pos[1] + FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_l_button_ > 0){
//                 sent_data_.target_pos[1] = received_data_.actual_pos[1] - FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_u_button_ > 0 ){
//                 sent_data_.target_pos[1] = received_data_.actual_pos[1] -THIRTY_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_d_button_ > 0){
//                 sent_data_.target_pos[1] = received_data_.actual_pos[1] + THIRTY_DEGREE_CCW/50 ;
//             }
//
//             if((controller_.left_r_button_ || controller_.left_l_button_ || controller_.left_u_button_ || controller_.left_d_button_)){
//                 sent_data_.control_word[1] = SM_GO_ENABLE;
//             }
//
//             // Settings for motor 3 
//             if(controller_.right_rb_button_ > 0 ){
//                 sent_data_.target_pos[2] = received_data_.actual_pos[2] + FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_rb_button_ > 0){
//                 sent_data_.target_pos[2] = received_data_.actual_pos[2] - FIVE_DEGREE_CCW/50 ;
//             }
//             if(controller_.left_start_button_ > 0 ){
//                 sent_data_.target_pos[2] = received_data_.actual_pos[2] + THIRTY_DEGREE_CCW/50 ;
//             }
//             if(controller_.right_start_button_ > 0){
//                 sent_data_.target_pos[2] = received_data_.actual_pos[2] - THIRTY_DEGREE_CCW/50 ;
//             }
//             if((controller_.right_rb_button_ || controller_.left_rb_button_ || controller_.left_start_button_ || controller_.right_start_button_)){
//                 sent_data_.control_word[2] = SM_GO_ENABLE;
//             }
//         }
//     }
// }

// CKim - Modifications
void EthercatLifeCycle::UpdateCyclicPositionModeParameters()
{
    float deadzone = 0.05;
    float amp = 1.0 - deadzone;
    float val;
    // printf( "Updating control parameters....\n");
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached || motor_state_[i]==kSwitchedOn)
        {
            // Settings for motor 1;
            val = controller_.left_x_axis_;
            if(val > deadzone) {
                sent_data_.target_pos[0] = received_data_.actual_pos[0] + (val-deadzone)/amp*THIRTY_DEGREE_CCW/50 ;
            }
            else if(val < -deadzone) {
                sent_data_.target_pos[0] = received_data_.actual_pos[0] + (val+deadzone)/amp*THIRTY_DEGREE_CCW/50 ;
            }
            // else {
            //     sent_data_.target_pos[0] = received_data_.actual_pos[0];
            // }
            //if((val > deadzone) || (val < -deadzone)){
                sent_data_.control_word[0] = SM_GO_ENABLE;
            //}

            // Settings for motor 2 
            val = controller_.right_x_axis_;
            if(val > deadzone) {
                sent_data_.target_pos[1] = received_data_.actual_pos[1] + (val-deadzone)/amp*THIRTY_DEGREE_CCW/50 ;
            }
            else if(val < -deadzone) {
                sent_data_.target_pos[1] = received_data_.actual_pos[1] + (val+deadzone)/amp*THIRTY_DEGREE_CCW/50 ;
            }
            // else {
            //     sent_data_.target_pos[1] = received_data_.actual_pos[1];
            // }
            //if((val > deadzone) || (val < -deadzone)){
                sent_data_.control_word[1] = SM_GO_ENABLE;
            //}

            // Settings for motor 3 
            if(controller_.right_rb_button_ > 0 ){
                sent_data_.target_pos[2] = received_data_.actual_pos[2] + FIVE_DEGREE_CCW/50/2.0 ;
            }
            else if(controller_.left_rb_button_ > 0){
                sent_data_.target_pos[2] = received_data_.actual_pos[2] - FIVE_DEGREE_CCW/50/2.0 ;
            }
            // else {
            //     sent_data_.target_pos[2] = received_data_.actual_pos[2];
            // }
            //if(controller_.right_rb_button_ || controller_.left_rb_button_){
                sent_data_.control_word[2] = SM_GO_ENABLE;
            //}
        }
    }
}

void EthercatLifeCycle::UpdateCyclicVelocityModeParameters() 
{
    float deadzone = 0.05;      
    float maxSpeed = 250.0;    // rpm
    float val;
    // printf( "Updating control parameters....\n");

    // Settings for motor 1;
    if(motor_state_[0]==kOperationEnabled || motor_state_[0]==kSwitchedOn)
    {
        val = controller_.left_y_axis_;
        if((val > deadzone) || (val < -deadzone))       
            {   sent_data_.target_vel[0] = -val*maxSpeed;    }
        else
            {   sent_data_.target_vel[0] = 0;               }
    }
    else    {   sent_data_.target_vel[0] = 0;   }
    sent_data_.control_word[0] = SM_GO_ENABLE;
    
    // Settings for motor 2 
    if(motor_state_[1]==kOperationEnabled || motor_state_[1]==kSwitchedOn)
    {
        val = controller_.left_x_axis_;
        if((val > deadzone) || (val < -deadzone))       
            {
                sent_data_.target_vel[1] = -val*maxSpeed;
                sent_data_.target_vel[0] -= sent_data_.target_vel[1];
            }
        else
            {   sent_data_.target_vel[1] = 0;               }
    }
    else    {   sent_data_.target_vel[1] = 0;   }
    sent_data_.control_word[1] = SM_GO_ENABLE;

    // Settings for motor 3 
    if(motor_state_[2]==kOperationEnabled || motor_state_[2]==kSwitchedOn)
    {
        if(controller_.right_rb_button_ > 0 )   {   sent_data_.target_vel[2] = 100;     }
        else if(controller_.left_rb_button_ > 0){   sent_data_.target_vel[2] = -100;    }
        else                                    {   sent_data_.target_vel[2] = 0;       }
    }
    else    {   sent_data_.target_vel[2] = 0;   }
    sent_data_.control_word[2] = SM_GO_ENABLE;




    // // Settings for motor 1;
    // if(motor_state_[0]==kOperationEnabled || motor_state_[0]==kSwitchedOn)
    // {
    //     if(controller_.green_button_)        {   sent_data_.target_vel[0] = 200;           }
    //     else if(controller_.yellow_button_)    {   sent_data_.target_vel[0] = -200;          }
    //     else                                {   sent_data_.target_vel[0] = 0;                           }
    // }
    // else    {   sent_data_.target_vel[0] = 0;   }
    // sent_data_.control_word[0] = SM_GO_ENABLE;
    
    // // Settings for motor 2 
    // if(motor_state_[1]==kOperationEnabled || motor_state_[1]==kSwitchedOn)
    // {
    //     if(controller_.red_button_)         
    //     {
    //         //sent_data_.target_vel[0] = -200;           
    //         sent_data_.target_vel[1] = 200;           
    //     }
    //     else if(controller_.blue_button_)   
    //     {   
    //         //sent_data_.target_vel[0] = +200;          
    //         sent_data_.target_vel[1] = -200;          
    //     }
    //     else {   sent_data_.target_vel[1] = 0;  }
    // }
    // else    {   sent_data_.target_vel[1] = 0;   }
    // sent_data_.control_word[1] = SM_GO_ENABLE;

    // // Settings for motor 3 
    // if(motor_state_[2]==kOperationEnabled || motor_state_[2]==kSwitchedOn)
    // {
    //     if(controller_.right_rb_button_ > 0 )   {   sent_data_.target_vel[2] = 100;     }
    //     else if(controller_.left_rb_button_ > 0){   sent_data_.target_vel[2] = -100;    }
    //     else                                    {   sent_data_.target_vel[2] = 0;       }
    // }
    // else    {   sent_data_.target_vel[2] = 0;   }
    // sent_data_.control_word[2] = SM_GO_ENABLE;
}

void EthercatLifeCycle::UpdateVelocityModeParameters() 
{
   // printf( "Updating control parameters....\n");
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached || motor_state_[i]==kSwitchedOn){
            if(controller_.right_x_axis_ > 1000 || controller_.right_x_axis_ < -1000 ){
                sent_data_.target_vel[0] = controller_.right_x_axis_ /150 ;
            }else{
                sent_data_.target_vel[0] = 0;
            }
            if(controller_.left_x_axis_ < -1000 || controller_.left_x_axis_ > 1000){
                sent_data_.target_vel[1] = controller_.left_x_axis_ /150 ;
            }else{
                sent_data_.target_vel[1] = 0 ;
            }
            if(controller_.left_y_axis_ < -1000 || controller_.left_y_axis_ > 1000){
                sent_data_.target_vel[2] = controller_.left_y_axis_ /150 ;
            }else{
                sent_data_.target_vel[2] = 0 ;
            }
        }else{
            sent_data_.target_vel[i]=0;
        }
    }

}

void EthercatLifeCycle::UpdateMotorStateVelocityMode()
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if ((received_data_.status_word[i] & command_) == 0X08){             
                //if status is fault, reset fault state.
                command_ = 0X04F;
                sent_data_.control_word[i] = SM_FULL_RESET;
                motor_state_[i] = kFault;
        }
        if(motor_state_[i]!=kSwitchedOn){
            sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
            if ( (received_data_.status_word[i] & command_) == 0x0040){  
                // If status is "Switch on disabled", \
                change state to "Ready to switch on"
                sent_data_.control_word[i]  = SM_GO_READY_TO_SWITCH_ON;
                command_ = 0x006f;
                motor_state_[i] = kSwitchOnDisabled;
            } else if ( (received_data_.status_word[i] & command_) == 0x0021){
                    // If status is "Ready to switch on", \
                        change state to "Switched on"
                sent_data_.control_word[i]  = SM_GO_SWITCH_ON;     
                command_ = 0x006f;
                motor_state_[i] = kReadyToSwitchOn;

            } else if ( (received_data_.status_word[i] & command_) == 0x0023){         
                // If status is "Switched on", change state to "Operation enabled"
                sent_data_.control_word[i]  = SM_GO_ENABLE;
                command_ = 0x006f;
                motor_state_[i] = kSwitchedOn;

            }else if ((received_data_.status_word[i] & command_) == 0X08){             
                //if status is fault, reset fault state.
                command_ = 0X04f;

                sent_data_.control_word[i]  = SM_FULL_RESET;
                motor_state_[i] = kFault;
            }
        }
    }
    return ;
}

void EthercatLifeCycle::EnableMotors()
{
    //DS402 CANOpen over EtherCAT state machine
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
        if ( (received_data_.status_word[i] & command_) == 0x0040){  
            // If status is "Switch on disabled", \
            change state to "Ready to switch on"
            sent_data_.control_word[i]  = SM_GO_READY_TO_SWITCH_ON;
            command_ = 0x006f;
            motor_state_[i] = kSwitchOnDisabled;
           // printf("Transiting to -Ready to switch on state...- \n");

        } else if ( (received_data_.status_word[i] & command_) == 0x0021){ // If status is "Ready to switch on", \
                                                        change state to "Switched on"
            sent_data_.control_word[i]  = SM_GO_SWITCH_ON;     
            command_ = 0x006f;
            motor_state_[i] = kReadyToSwitchOn;
         //   printf("Transiting to -Switched on state...- \n");        

        } else if ( (received_data_.status_word[i] & command_) == 0x0023){         
            // If status is "Switched on", change state to "Operation enabled"

            // printf("Operation enabled...\n");
            sent_data_.control_word[i]  = SM_GO_ENABLE;
            command_ = 0x006f;
            motor_state_[i] = kSwitchedOn;

        }else if ((received_data_.status_word[i] & command_) == 0X08){             
            //if status is fault, reset fault state.
            command_ = 0X04F;

            sent_data_.control_word[i] = SM_FULL_RESET;
            motor_state_[i] = kFault;

        }
    }
}
