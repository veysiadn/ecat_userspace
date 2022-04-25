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
        printf( "Configuration phase failed\n");
        return FAILURE;
    }else{
        return SUCCESS;
    }
}

uint8_t EthercatLifeCycle::on_activate()
{
    if(StartEthercatCommunication()){

        printf("Activation phase failed");
        return FAILURE;
    }else{
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
    pthread_cancel(ethercat_thread_);
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
    // CPU_SET(8,&mask);
    // CPU_SET(9,&mask);
	// CPU_SET(10,&mask);
	// CPU_SET(11,&mask);

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

    if(SetConfigurationParameters()){
        printf("Configuration parameters set failed\n");
        return -1 ;
    }

    printf("Mapping default PDOs...\n");
    if(ecat_node_->MapDefaultPdos()){
        return  -1 ;
    }

    printf("Configuring DC synchronization...\n");
    if(DISTRIBUTED_CLOCK){
     ecat_node_->ConfigDcSyncDefault();
    }
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
int EthercatLifeCycle::SetConfigurationParameters()
{
    #if VELOCITY_MODE
        ProfileVelocityParam P ;
        
        P.profile_acc=3e4 ;
        P.profile_dec=3e4 ;
        P.max_profile_vel = 1000 ;
        P.quick_stop_dec = 3e4 ;
        P.motion_profile_type = 0 ;
        return ecat_node_->SetProfileVelocityParametersAll(P);
    #endif

    #if POSITION_MODE
        ProfilePosParam P ;
        uint32_t max_fol_err ;
        P.profile_vel = 450; 
        P.profile_acc = 1e4;
        P.profile_dec = 1e4;
        P.max_profile_vel = 500;
        P.quick_stop_dec = 3e4;
        P.motion_profile_type = 0 ;
        return ecat_node_->SetProfilePositionParametersAll(P);
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
        return ecat_node_->SetCyclicSyncPositionModeParametersAll(P);
    #endif

    #if CYCLIC_VELOCITY_MODE
        printf("Setting drives to CSV mode...\n");
        CSVelocityModeParam P ;
        P.velocity_controller_gain.Pgain = 40000;
        P.velocity_controller_gain.Igain = 800000;
        P.profile_dec=3e4 ;
        P.quick_stop_dec = 3e4 ;
        P.interpolation_time_period = 0;//1 ;
        return ecat_node_->SetCyclicSyncVelocityModeParametersAll(P);
    #endif

    #if CYCLIC_TORQUE_MODE
        printf("Setting drives to CST mode...\n");
        CSTorqueModeParam P ;
        P.profile_dec=3e4 ;
        P.quick_stop_dec = 3e4 ;
        return ecat_node_->SetCyclicSyncTorqueModeParametersAll(P);
    #endif
}
int  EthercatLifeCycle::StartEthercatCommunication()
{
    err_= pthread_create(&ethercat_thread_,&ethercat_thread_attr_, &EthercatLifeCycle::PassCycylicExchange,this);
    if(err_)
    {
        printf( "Error : Couldn't start communication thread.!\n");
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
    uint32_t print_val = 1e4;
    int error_check=0;
    uint8_t sync_ref_counter = 1;
    struct timespec wake_up_time, time, publish_time_start={}, publish_time_end={};
    int begin=1e4;
    int status_check_counter = 1000;
    
    // ------------------------------------------------------- //
    // CKim - Initialization loop before entring control loop. 
    // Switch On and Enable Driver
    printf( "Enabling motors...\n");
    // get current time
    clock_gettime(CLOCK_TO_USE, &wake_up_time);
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
            printf( "All drives enabled\n");
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
                printf( "Connection error, check your physical connection.\n");
                al_state_ = g_master_state.al_states ; 
                received_data_.emergency_switch_val=0;
                emergency_status_=0;
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
                    printf( "Trying to enable motors\n");
                } 
            }
        }
#if POSITION_MODE
        WriteToSlavesInPositionMode();
#endif 
#if CYCLIC_POSITION_MODE
        WriteToSlavesInPositionMode();
#endif 
#if VELOCITY_MODE       
        WriteToSlavesVelocityMode();
#endif
#if CYCLIC_VELOCITY_MODE       
        WriteToSlavesVelocityMode();
#endif
#if CYCLIC_TORQUE_MODE
        WriteToSlavesInCyclicTorqueMode();
#endif
        // CKim - Sync Timer
        if (sync_ref_counter) {
            sync_ref_counter--;
        } else {
            sync_ref_counter = 1; // sync every cycle

            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(time));
        }
        ecrt_master_sync_slave_clocks(g_master);

        ecrt_domain_queue(g_master_domain);
        // CKim - Send process data
        ecrt_master_send(g_master);
    }// while(sig)
    printf( "All motors enabled, entering control loop\n");

    // ------------------------------------------------------- //
    // CKim - All motors enabled. Start control loop
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

    while(sig){
        wake_up_time = timespec_add(wake_up_time, g_cycle_time);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
        ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));
        
        #if MEASURE_TIMING
            clock_gettime(CLOCK_TO_USE, &start_time);
            timer_info_.GetTime();
            timer_info_.MeasureTimeDifference();
            old_latency = latency_ns;
            latency_ns = DIFF_NS(wake_up_time, start_time);
            period_ns = DIFF_NS(last_start_time, start_time);
            exec_ns = DIFF_NS(last_start_time, end_time);
            last_start_time = start_time;
            if(!begin)
            {
                jitter = latency_ns - old_latency ;
                if(jitter < 0 )                     jitter *=-1; 
                if(jitter > jitter_max)             jitter_max  = jitter ;
                if(jitter < jitter_min)             jitter_min  = jitter;                 
                if(period_ns > max_period)          max_period  = period_ns;
                if(period_ns < min_period)          min_period  = period_ns;                
                if(exec_ns > exec_max)              exec_max    = exec_ns;
                if(exec_ns < exec_min)              exec_min    = exec_ns;
                if(latency_ns > max_latency)        max_latency = latency_ns;
                if(latency_ns < latency_min)        latency_min = latency_ns;
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
                printf( "Connection error, check your physical connection.\n");
                al_state_ = g_master_state.al_states ; 
                received_data_.emergency_switch_val=0;
                emergency_status_=0;
                //PublishAllData();
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
            // output timing stats
            if(!print_val){
                    printf("-----------------------------------------------\n\n");
                    printf("Tperiod   min   : %10u ns  | max : %10u ns\n",
                            period_min_ns, period_max_ns);
                    printf("Texec     min   : %10u ns  | max : %10u ns\n",
                            exec_min_ns, exec_max_ns);
                    printf("Tlatency  min   : %10u ns  | max : %10u ns\n",
                            latency_min_ns, latency_max_ns);
                    printf("Tjitter max     : %10u ns  \n",
                            latency_max_ns-latency_min_ns);
                    printf("-----------------------------------------------\n\n");       
                    printf("Tperiod min     : %10u ns  | max : %10u ns\n",
                            min_period, max_period);
                    printf("Texec  min      : %10u ns  | max : %10u ns\n",
                            exec_min, exec_max);
                    printf("Tjitter min     : %10u ns  | max : %10u ns\n",
                            jitter_min, jitter_max);  
                    printf("Timer info values: %10d us \n", timer_info_.time_span_.count());                             
                    printf("-----------------------------------------------\n\n");
                    print_val=1000;

            }else {
                print_val--;
            }      
                period_max_ns = 0;
                period_min_ns = 0xffffffff;

                exec_max_ns = 0;
                exec_min_ns = 0xffffffff;

                latency_max_ns = 0;
                latency_min_ns = 0xffffffff;
        #endif
        ReadFromSlaves();
#if POSITION_MODE
        UpdatePositionModeParameters();
        WriteToSlavesInPositionMode();
#endif
#if CYCLIC_POSITION_MODE
        UpdateCyclicPositionModeParameters();
        WriteToSlavesInPositionMode();
#endif 
#if VELOCITY_MODE
        UpdateVelocityModeParameters();
        WriteToSlavesVelocityMode();
#endif
#if CYCLIC_VELOCITY_MODE
        UpdateCyclicVelocityModeParameters();
        WriteToSlavesVelocityMode();
#endif

#if CYCLIC_TORQUE_MODE
        UpdateCyclicTorqueModeParameters();
        WriteToSlavesInCyclicTorqueMode();
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

    printf("Leaving control thread.\n");
    ecat_node_->DeactivateCommunication();
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);
    pthread_cancel(ethercat_thread_);
    pthread_exit(NULL);
    return;
}// StartPdoExchange end

void EthercatLifeCycle::ReadFromSlaves()
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        received_data_.actual_pos[i]  = EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_pos);
        received_data_.actual_vel[i]  = EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_vel);
        received_data_.status_word[i] = EC_READ_U16(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.status_word);
        received_data_.actual_tor[i]  = EC_READ_S16(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_tor);
    }
    received_data_.com_status = al_state_ ; 
    #if CUSTOM_SLAVE
        received_data_.right_limit_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.r_limit_switch);
        received_data_.left_limit_switch_val  = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.l_limit_switch);
        received_data_.emergency_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.emergency_switch);
        emergency_status_  = received_data_.emergency_switch_val;
    #endif
}// ReadFromSlaves end

void EthercatLifeCycle::WriteToSlavesVelocityMode()
{
  //  printf( "Writing to slaves....\n");
    emergency_status_=1;
  if(!emergency_status_ ){
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


int EthercatLifeCycle::GetComState()
{
    return al_state_ ; 
}

void EthercatLifeCycle::UpdatePositionModeParameters()
{   
    /// WRITE YOUR CUSTOM CONTROL ALGORITHM, VARIABLES DECLARATAION HERE, LIKE IN EXAMPLE BELOW.
    /// KEEP IN MIND THAT YOU WILL HAVE TO WAIT FOR THE MOTION TO FINISH IN POSITION MODE, THEREFORE
    /// YOU HAVE TO CHECK 10th BIT OF STATUS WORD TO CHECK WHETHER TARGET IS REACHED OR NOT.
    static uint8_t operation_ready = 0 ;
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if(motor_state_[i]==kOperationEnabled || 
        motor_state_[i]==kTargetReached || motor_state_[i]==kSwitchedOn){
            if (controller_.xbox_button_){
                for(int j = 0 ; j < g_kNumberOfServoDrivers ; j++){
                    sent_data_.target_pos[j] = 0 ; 
                    if(!operation_ready){
                         sent_data_.control_word[j] = SM_RUN ;
                         if(TEST_BIT(received_data_.status_word[j],10)){
                            operation_ready = 1; 
                         }
                    }else{
                        sent_data_.control_word[0] = SM_GO_ENABLE;
                        operation_ready = 0; 
                    }
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
            
            if(controller_.red_button_ || controller_.blue_button_ || controller_.green_button_ 
            || controller_.yellow_button_){
                sent_data_.control_word[0] = SM_GO_ENABLE;
                if(!operation_ready){
                    sent_data_.control_word[0] = SM_RUN ;
                    if(TEST_BIT(received_data_.status_word[0],10)){
                    operation_ready = 1; 
                    }
                }else{
                    sent_data_.control_word[0] = SM_GO_ENABLE;
                    operation_ready = 0; 
                }
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
    for(int i = 0 ; i < g_kNumberOfServoDrivers; i++){
            EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
            EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_pos,sent_data_.target_pos[i]);
    }
}

void EthercatLifeCycle::WriteToSlavesInCyclicTorqueMode()
{
  //  printf( "Writing to slaves....\n");
    emergency_status_=1;
  if(!emergency_status_)
  {
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
        EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_tor,0);
        EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.torque_offset,0);
        
    }
  }
  else
  {
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
        EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_tor,sent_data_.target_tor[i]);
        EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.torque_offset,0);
    }
  }
}

// CKim - Modifications
void EthercatLifeCycle::UpdateCyclicPositionModeParameters()
{
    /// WRITE YOUR CUSTOM CONTROL ALGORITHM, VARIABLES DECLARATAION HERE, LIKE IN EXAMPLE BELOW.
    float deadzone = 0.05;
    float amp = 1.0 - deadzone;
    float val;
    static uint8_t operation_ready = 0;
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached 
        || motor_state_[i]==kSwitchedOn)
        {
            // Settings for motor 1;
            val = controller_.left_x_axis_;
            if(val > deadzone) {
                sent_data_.target_pos[0] = received_data_.actual_pos[0] + (val-deadzone)/amp*THIRTY_DEGREE_CCW/500 ;
            }
            else if(val < -deadzone) {
                sent_data_.target_pos[0] = received_data_.actual_pos[0] + (val+deadzone)/amp*THIRTY_DEGREE_CCW/500 ;
            }
            // else {
            //     sent_data_.target_pos[0] = received_data_.actual_pos[0];
            // }
            //if((val > deadzone) || (val < -deadzone)){
            if (!operation_ready){
                sent_data_.control_word[0] = SM_RUN;
                if(TEST_BIT(received_data_.status_word[0],10))
                    operation_ready = 1;
            }else{
                sent_data_.control_word[0] = SM_GO_ENABLE;
                operation_ready = 0; 
            }
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
    /// WRITE YOUR CUSTOM CONTROL ALGORITHM, VARIABLES DECLARATAION HERE, LIKE IN EXAMPLE BELOW.
    float deadzone = 0.05;      
    float maxSpeed = 250.0;
    float val;
    if(motor_state_[0]==kOperationEnabled || motor_state_[0]==kSwitchedOn)
    {
        val = controller_.left_y_axis_;
        if((val > deadzone) || (val < -deadzone))       
            {   sent_data_.target_vel[0] = int32_t(-val*maxSpeed);  }
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
    /// WRITE YOUR CUSTOM CONTROL ALGORITHM VARIABLES DECLARATAION HERE
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached 
            || motor_state_[i]==kSwitchedOn){
               /// WRITE YOUR CUSTOM CONTROL ALGORITHM HERE IF YOU WANT TO USE VELOCITY MODE
              /// YOU CAN CHECK  EXAMPLE CONTROL CODE BELOW.
            if(controller_.right_x_axis_ > 0.1 || controller_.right_x_axis_ < -0.1 ){
                sent_data_.target_vel[0] = controller_.right_x_axis_ *1000 ;
            }else{
                sent_data_.target_vel[0] = 0;
            }
            if(controller_.left_x_axis_ < -0.1 || controller_.left_x_axis_ > 0.1){
                sent_data_.target_vel[1] = controller_.left_x_axis_ *1000 ;
            }else{
                sent_data_.target_vel[1] = 0 ;
            }
            if(controller_.left_y_axis_ < -0.1 || controller_.left_y_axis_ > 0.1){
                sent_data_.target_vel[2] = controller_.left_y_axis_ *1000 ;
            }else{
                sent_data_.target_vel[2] = 0 ;
            }
        }else{
            sent_data_.target_vel[i]=0;
        }
    }

}

void EthercatLifeCycle::UpdateCyclicTorqueModeParameters()
{
    /// WRITE YOUR CUSTOM CONTROL ALGORITHM VARIABLES DECLARATAION HERE
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        sent_data_.control_word[i] = SM_GO_ENABLE ; 
        if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached || motor_state_[i]==kSwitchedOn){
              /// WRITE YOUR CUSTOM CONTROL ALGORITHM HERE IF YOU WANT TO USE CYCLIC TORQUE MODE
              /// YOU CAN CHECK  EXAMPLE CONTROL CODE BELOW.
            if(controller_.right_x_axis_ > 0.1 || controller_.right_x_axis_ < -5000 ){
                sent_data_.target_tor[0] = controller_.left_x_axis_ * 500 ;
            }else{
                sent_data_.target_tor[0] = 0;
            }
            if(controller_.left_x_axis_ < -0.1 || controller_.left_x_axis_ > 0.1){
                sent_data_.target_tor[1] = controller_.left_x_axis_ *500;
            }else{
                sent_data_.target_tor[1] = 0 ;
            }
            if(controller_.left_y_axis_ < -0.1 || controller_.left_y_axis_ > 0.1){
                sent_data_.target_tor[2] = controller_.left_y_axis_ *500 ;
            }else{
                sent_data_.target_tor[2] = 0 ;
            }
        }else{
            sent_data_.target_tor[i]=0;
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
                // std::cout << "Sending go reeady to switch on" << std::endl;
            } else if ( (received_data_.status_word[i] & command_) == 0x0021){
                    // If status is "Ready to switch on", \
                        change state to "Switched on"
                sent_data_.control_word[i]  = SM_GO_SWITCH_ON;     
                command_ = 0x006f;
                motor_state_[i] = kReadyToSwitchOn;
                // std::cout << "Sending go switch on" << std::endl;
                
            } else if ( (received_data_.status_word[i] & command_) == 0x0023){         
                // If status is "Switched on", change state to "Operation enabled"
                sent_data_.control_word[i]  = SM_GO_ENABLE;
                command_ = 0x006f;
                motor_state_[i] = kSwitchedOn;
                // std::cout << "Sending go enable" << std::endl;                

            }else if ((received_data_.status_word[i] & command_) == 0X08){             
                //if status is fault, reset fault state.
                command_ = 0X04f;

                sent_data_.control_word[i]  = SM_FULL_RESET;
                motor_state_[i] = kFault;
                // std::cout << "Sending full reset" << std::endl;                
            }
        }
    }
    return ;
}