Search.setIndex({docnames:["core","endpoint_definitions_guide","example","index","installation","new_peripheral_guide","peripherals","register_index_guide","tests","utils"],envversion:{"sphinx.domains.c":2,"sphinx.domains.changeset":1,"sphinx.domains.citation":1,"sphinx.domains.cpp":4,"sphinx.domains.index":1,"sphinx.domains.javascript":2,"sphinx.domains.math":2,"sphinx.domains.python":3,"sphinx.domains.rst":2,"sphinx.domains.std":2,"sphinx.ext.intersphinx":1,"sphinx.ext.todo":2,"sphinx.ext.viewcode":1,sphinx:56},filenames:["core.rst","endpoint_definitions_guide.rst","example.rst","index.rst","installation.rst","new_peripheral_guide.rst","peripherals.rst","register_index_guide.rst","tests.rst","utils.rst"],objects:{"pyripherals.core":[[0,1,1,"","Endpoint"],[0,1,1,"","FPGA"],[0,1,1,"","Register"],[0,3,1,"","disp_device"]],"pyripherals.core.Endpoint":[[0,2,1,"","advance_endpoints"],[0,2,1,"","excel_to_defines"],[0,2,1,"","get_chip_endpoints"],[0,2,1,"","update_endpoints_from_defines"]],"pyripherals.core.FPGA":[[0,2,1,"","clear_endpoint"],[0,2,1,"","clear_wire_bit"],[0,2,1,"","init_device"],[0,2,1,"","read_ep"],[0,2,1,"","read_pipe_out"],[0,2,1,"","read_trig"],[0,2,1,"","read_wire"],[0,2,1,"","read_wire_bit"],[0,2,1,"","send_trig"],[0,2,1,"","set_endpoint"],[0,2,1,"","set_ep_simultaneous"],[0,2,1,"","set_wire"],[0,2,1,"","set_wire_bit"],[0,2,1,"","toggle_high"],[0,2,1,"","toggle_low"]],"pyripherals.core.Register":[[0,2,1,"","get_chip_registers"]],"pyripherals.peripherals":[[6,0,0,"-","AD5453"],[6,0,0,"-","AD7961"],[6,0,0,"-","ADCDATA"],[6,0,0,"-","ADS8686"],[6,0,0,"-","DAC53401"],[6,0,0,"-","DAC80508"],[6,0,0,"-","DDR3"],[6,0,0,"-","I2CController"],[6,0,0,"-","SPIController"],[6,0,0,"-","SPIFifoDriven"],[6,0,0,"-","TCA9555"],[6,0,0,"-","TMF8801"],[6,0,0,"-","UID_24AA025UID"]],"pyripherals.peripherals.AD5453":[[6,1,1,"","AD5453"]],"pyripherals.peripherals.AD5453.AD5453":[[6,2,1,"","read_coeff_debug"],[6,2,1,"","set_clk_rising_edge"],[6,2,1,"","set_ctrl_reg"]],"pyripherals.peripherals.AD7961":[[6,1,1,"","AD7961"]],"pyripherals.peripherals.AD7961.AD7961":[[6,2,1,"","create_chips"],[6,2,1,"","get_fifo_status"],[6,2,1,"","get_pll_status"],[6,2,1,"","get_status"],[6,2,1,"","get_timing_pll_status"],[6,2,1,"","power_down_adc"],[6,2,1,"","power_down_all"],[6,2,1,"","power_down_fpga"],[6,2,1,"","reset_pll"],[6,2,1,"","reset_trig"],[6,2,1,"","reset_wire"],[6,2,1,"","set_enables"],[6,2,1,"","setup"],[6,2,1,"","test_pattern"]],"pyripherals.peripherals.ADS8686":[[6,1,1,"","ADS8686"]],"pyripherals.peripherals.ADS8686.ADS8686":[[6,2,1,"","hw_reset"],[6,2,1,"","read"],[6,2,1,"","read_channel"],[6,2,1,"","read_last"],[6,2,1,"","reg_to_voltage"],[6,2,1,"","set_range"],[6,2,1,"","setup"],[6,2,1,"","setup_sequencer"],[6,2,1,"","write"],[6,2,1,"","write_reg_bridge"]],"pyripherals.peripherals.DAC53401":[[6,1,1,"","DAC53401"]],"pyripherals.peripherals.DAC53401.DAC53401":[[6,2,1,"","config_func"],[6,2,1,"","config_margins"],[6,2,1,"","config_rate"],[6,2,1,"","config_step"],[6,2,1,"","enable_internal_reference"],[6,2,1,"","get_gain"],[6,2,1,"","get_id"],[6,2,1,"","lock"],[6,2,1,"","power_down_10k"],[6,2,1,"","power_down_high_impedance"],[6,2,1,"","power_up"],[6,2,1,"","read"],[6,2,1,"","reset"],[6,2,1,"","set_gain"],[6,2,1,"","start_func"],[6,2,1,"","stop_func"],[6,2,1,"","unlock"],[6,2,1,"","write"],[6,2,1,"","write_voltage"]],"pyripherals.peripherals.DAC80508":[[6,1,1,"","DAC80508"]],"pyripherals.peripherals.DAC80508.DAC80508":[[6,2,1,"","reset"],[6,2,1,"","set_config"],[6,2,1,"","set_config_bin"],[6,2,1,"","set_gain"],[6,2,1,"","set_gain_bin"],[6,2,1,"","write_chip_reg"],[6,2,1,"","write_voltage"]],"pyripherals.peripherals.DDR3":[[6,1,1,"","DDR3"]],"pyripherals.peripherals.DDR3.DDR3":[[6,2,1,"","adc_single"],[6,2,1,"","clear_adc_debug"],[6,2,1,"","clear_adc_read"],[6,2,1,"","clear_adc_write"],[6,2,1,"","clear_adcs_connected"],[6,2,1,"","clear_dac_read"],[6,2,1,"","clear_dac_write"],[6,2,1,"","closest_frequency"],[6,2,1,"","data_to_names"],[6,2,1,"","deswizzle"],[6,2,1,"","fifo_status"],[6,2,1,"","make_flat_voltage"],[6,2,1,"","make_ramp"],[6,2,1,"","make_sine_wave"],[6,2,1,"","make_step"],[6,2,1,"","print_fifo_status"],[6,2,1,"","read_adc"],[6,2,1,"","read_adc_block"],[6,2,1,"","reset_fifo"],[6,2,1,"","reset_mig_interface"],[6,2,1,"","save_data"],[6,2,1,"","set_adc_dac_simultaneous"],[6,2,1,"","set_adc_debug"],[6,2,1,"","set_adc_read"],[6,2,1,"","set_adc_write"],[6,2,1,"","set_adcs_connected"],[6,2,1,"","set_dac_read"],[6,2,1,"","set_dac_write"],[6,2,1,"","set_index"],[6,2,1,"","write"],[6,2,1,"","write_channels"]],"pyripherals.peripherals.I2CController":[[6,1,1,"","I2CController"]],"pyripherals.peripherals.I2CController.I2CController":[[6,2,1,"","create_chips"],[6,2,1,"","i2c_configure"],[6,2,1,"","i2c_read_long"],[6,2,1,"","i2c_receive"],[6,2,1,"","i2c_transmit"],[6,2,1,"","i2c_write_long"],[6,2,1,"","reset_device"]],"pyripherals.peripherals.SPIController":[[6,1,1,"","SPIController"]],"pyripherals.peripherals.SPIController.SPIController":[[6,2,1,"","configure_master"],[6,2,1,"","configure_master_bin"],[6,2,1,"","create_chips"],[6,2,1,"","get_master_configuration"],[6,2,1,"","read"],[6,2,1,"","reset_master"],[6,2,1,"","select_slave"],[6,2,1,"","set_divider"],[6,2,1,"","set_fpga_mode"],[6,2,1,"","set_frequency"],[6,2,1,"","set_host_mode"],[6,2,1,"","wb_go"],[6,2,1,"","wb_is_ack"],[6,2,1,"","wb_read"],[6,2,1,"","wb_send_cmd"],[6,2,1,"","wb_set_address"],[6,2,1,"","wb_write"],[6,2,1,"","write"]],"pyripherals.peripherals.SPIFifoDriven":[[6,1,1,"","SPIFifoDriven"]],"pyripherals.peripherals.SPIFifoDriven.SPIFifoDriven":[[6,2,1,"","create_chips"],[6,2,1,"","filter_downsample"],[6,2,1,"","filter_select"],[6,2,1,"","filter_sum"],[6,2,1,"","set_clk_divider"],[6,2,1,"","set_ctrl_reg"],[6,2,1,"","set_data_mux"],[6,2,1,"","set_spi_sclk_divide"],[6,2,1,"","write"]],"pyripherals.peripherals.TCA9555":[[6,1,1,"","TCA9555"]],"pyripherals.peripherals.TCA9555.TCA9555":[[6,2,1,"","configure_pins"],[6,2,1,"","read"],[6,2,1,"","write"]],"pyripherals.peripherals.TMF8801":[[6,1,1,"","TMF8801"]],"pyripherals.peripherals.TMF8801.TMF8801":[[6,2,1,"","factory_calibration"],[6,2,1,"","get_id"],[6,2,1,"","power_down_high_impedance"],[6,2,1,"","read"],[6,2,1,"","read_data"],[6,2,1,"","write"]],"pyripherals.peripherals.UID_24AA025UID":[[6,1,1,"","UID_24AA025UID"]],"pyripherals.peripherals.UID_24AA025UID.UID_24AA025UID":[[6,2,1,"","get_device_code"],[6,2,1,"","get_manufacturer_code"],[6,2,1,"","get_serial_number"],[6,2,1,"","read"],[6,2,1,"","write"]],"pyripherals.utils":[[9,3,1,"","calc_impedance"],[9,3,1,"","count_bytes"],[9,3,1,"","create_yaml"],[9,3,1,"","from_voltage"],[9,3,1,"","get_memory_usage"],[9,3,1,"","int_to_list"],[9,3,1,"","read_h5"],[9,3,1,"","reverse_bits"],[9,3,1,"","to_voltage"],[9,3,1,"","twos_comp"]],pyripherals:[[0,0,0,"-","core"],[9,0,0,"-","utils"]]},objnames:{"0":["py","module","Python module"],"1":["py","class","Python class"],"2":["py","method","Python method"],"3":["py","function","Python function"]},objtypes:{"0":"py:module","1":"py:class","2":"py:method","3":"py:function"},terms:{"0":[0,6,7,9],"010":6,"0x":[1,7],"0x0":7,"0x00":7,"0x000":7,"0x0000":7,"0x01":7,"0x02":7,"0x04":1,"0x10":7,"0x1d":6,"0x1e":6,"0x1f":6,"0x20":6,"0x21":6,"0x22":6,"0x23":6,"0x24":6,"0x25":6,"0x26":6,"0x27":6,"0x29":6,"0x3010":6,"0x31a0":7,"0x3f":7,"0x41":6,"0x44":7,"0x5555":6,"0xaaaa":6,"0xffff":7,"1":[0,6,9],"10":[6,7],"100":6,"1000":6,"1024":0,"1024word":6,"10k":6,"11":[],"12304":6,"128":6,"12824":6,"14":6,"144":[],"15":7,"16":[0,1,6,7],"16384":6,"1st":6,"2":[6,9],"2021":[],"2022":[0,9],"2048":6,"24":6,"24aa025uid":6,"256":6,"256bit":6,"2nd":6,"2x":6,"3":[0,6],"30":6,"32":[4,6],"32bit":6,"34":6,"376":6,"3v":0,"3x":6,"4":6,"40":6,"4096":6,"4294967295":0,"47":6,"4x":6,"5":6,"50":6,"54":[],"5x":6,"6":6,"64":6,"65535":6,"7":[6,7],"8":[1,6,7,9],"8192":6,"8th":6,"9":7,"boolean":6,"byte":[0,6,9],"class":[0,4,5,6,9],"default":[0,6,9],"do":[6,7,8],"final":6,"float":[6,9],"function":[5,6,9],"i\u00b2c":[],"import":4,"int":[0,6,9],"long":9,"new":[3,4,6,7],"return":[0,6,9],"short":5,"static":[0,6],"true":[0,1,6],"while":[1,6],A:6,AND:6,And:[],At:6,Be:[5,7],For:[1,2,5,6,7],IN:6,If:[1,4,5,6],In:7,Into:[],It:9,NOT:1,No:6,Not:6,One:2,TO:[],That:6,The:[0,1,2,5,6,7,9],Then:9,These:[1,6],To:[1,4,5,6],With:[2,5],_:1,_gen_addr:1,_gen_bit:1,a3:6,ab:[0,9],about:[0,1],abov:[1,5,8],access:7,accord:1,acknowledg:6,across:2,activ:[0,6],actual:6,actual_frequ:6,actual_length:6,ad5453:6,ad7961:[0,6],ad7961_ch0:6,ad7961_ch1:6,ad7961_ch2:6,ad7961_ch3:6,ad7961_tim:6,ad796x:6,ad:[1,5,6,7],adc:[6,9],adc_data:[6,9],adc_data_count:6,adc_no_timestamp:6,adc_read:6,adc_singl:6,adc_transfer_en:6,adc_write_en:6,adcdata:[],add:[1,4,5,6],addit:5,addr:[0,1],addr_pin:6,address:[0,6],address_head:6,ads8686:[2,6],ads8686_cha:6,ads8686_chb:6,advanc:0,advance_endpoint:[0,1],advance_endpoints_bynum:[],advance_num:0,after:[0,1,4,6],again:6,ajstr:0,ajstroschein:[0,9],aldo:6,all:[0,4,6,8],allow:[1,6,7],alm_en:6,alm_sel:6,along:6,alongsid:6,alreadi:7,also:[1,2,4,5,6],altern:1,ambient:6,amplitud:6,an000597:6,an:[0,1,2,4,5,6,8,9],analyz:2,ani:[0,1,5,6,7],anoth:[1,6],api:[0,6],append:6,applic:5,ar:[1,2,4,5,6,7,8],argument:6,around:6,arrai:[6,9],ass:6,assign:6,associ:1,assum:9,attribut:[0,6],august:[],auto_gain:6,autom:8,automat:[1,6],avail:[4,6,7,8],avdd:6,avoidi:5,b:6,back:[0,2,6],base:[],basic:6,becaus:1,been:0,befor:[1,6],begin:[],belong:1,below:[1,4,5,6,7],between:[2,6],big:9,binari:[6,9],bit:[0,2,4,6,9],bit_index_high:0,bit_index_low:0,bit_list:0,bit_width:[0,1,9],bitfil:[0,2],bitfile_path:2,blk_multipl:6,blob:[],block:6,block_pipe_return:6,block_siz:6,bool:[0,6],both:[0,1,6,9],branch:5,bu:0,buf:6,buffer:[0,6],byearrai:6,bytearrai:[0,6],byteord:9,bytes_read_error:6,c:[0,4],calc_imped:9,calcul:[2,6,9],calibr:6,call:[1,6,7],can:[1,2,4,6,7,8],cannot:[6,7],captur:6,categori:1,cell:1,chan_data:6,chan_list:[6,9],chan_num:6,chang:[1,2,6,7],channel:[6,9],char_len:6,check:[0,1,5,6],chip:[0,1,6],chip_nam:0,chipnam:1,circuit:2,cl:[],classmethod:6,clear:[0,6],clear_adc_debug:6,clear_adc_read:6,clear_adc_writ:6,clear_adcs_connect:6,clear_dac_read:6,clear_dac_writ:6,clear_endpoint:0,clear_read:[],clear_wire_bit:0,clear_writ:[],clk:6,clk_div:6,clock:6,closest:6,closest_frequ:6,cm:6,cmd:6,code:[0,5,6],code_block:[],coeffici:6,collabor:[],collis:0,column:[1,7],com:6,combin:6,come:6,command:[5,6,8],comment:[1,5],common:6,commonli:5,commun:[0,5,6],complement:[6,9],complet:[1,5,6,7],compon:[2,9],comput:[2,9],concern:7,config:[4,7,9],config_func:6,config_margin:6,config_r:6,config_step:6,configur:[0,6],configure_mast:6,configure_master_bin:6,configure_pin:6,connect:[0,2,6,8,9],connectedto:[],consid:5,consol:[],constant:[2,6],contain:[0,1,6,7,9],containt:0,content:[],continu:6,contribut:5,control:1,convers:6,convert:[0,6,9],convert_data:[],convert_two:6,copi:[0,1,6],core:[3,6],correct:6,correctli:6,correspoind:6,correspond:[6,7],count:[6,9],count_byt:9,counter:6,covg_fpga:0,crc_en:6,creat:[0,1,4,5,6,7,9],create_chip:[5,6],create_yaml:[4,9],ctrl:6,ctrlvalu:6,current:[2,4,6],current_data_mux:6,custom:4,cycl:6,d:6,dac0_pwdwn:6,dac1_pwdwn:6,dac2_pwdwn:6,dac3_pwdwn:6,dac4_pwdwn:6,dac53401:6,dac5_pwdwn:6,dac6_pwdwn:6,dac7_pwdwn:6,dac80508:[2,5,6],dac:6,dac_data:6,dac_read_en:6,dac_write_en:6,daq_v2:[],dark:6,data:[0,1,6,7,9],data_buf:6,data_dir:[6,9],data_len:0,data_length:6,data_mux:6,data_to_nam:6,data_vers:6,datasheet:[5,6,7],date:5,ddr3:[2,6],ddr:6,ddr_norepeat:6,ddr_seq:6,debug:[0,6],debug_print:6,decid:6,decim:[1,6,7],declar:1,default_data_mux:6,defin:[1,6],defines_path:0,definit:[0,3,7],deriv:[0,6],descript:[5,6],design:6,desir:6,deswizl:[],deswizzl:6,determin:[6,9],dev:0,devaddr:6,devic:[0,4,6],device_info:0,dicionari:9,dict:[0,6],dictionari:[0,1,6],differ:[1,6,8],differenti:6,digit:6,direct:6,directli:6,directori:6,disp_devic:0,displai:0,div_ref:6,divid:[6,9],divide_refer:6,divide_valu:6,doc:6,docstr:5,document:2,doe:[1,5,6,7],don:6,done:[5,6],doubl:6,down:6,download:[2,4],downsampl:6,driven:6,dsdo:6,duplic:5,duti:6,each:[0,1,6,7],edg:6,edit:4,edu:[0,9],either:[1,6,9],empti:[0,1,6],emul:6,en0:6,en:6,enabl:6,enable_internal_refer:6,endian:9,endpoint:[0,3,6,7],endpoint_definitions_guid:[],endpoint_max_width:4,endpoint_nam:1,endpoints_dict:0,endpoints_from_defin:[0,6],enter:[1,6],environ:6,ep:[],ep_bit:0,ep_defin:[0,1,4],ep_defines_exampl:[],ep_defines_path:[0,4],ep_defines_sheet_exampl:[],ep_defines_sheet_templ:[],error:[0,1,6],even:6,evenli:6,everyth:[],ex:1,examp:5,exampl:[0,1,3,4,5,7],excel:[0,1,7],excel_path:0,excel_to_defin:[0,1],expand:6,expect:[0,6],explain:[1,5],extend:5,extens:6,extra:[1,7],extract:1,factor2:6,factor:6,factori:6,factory_calibr:6,fail:[0,6],fall:6,fals:[0,1,6,9],fast:6,few:6,fg:6,field:6,fifo:6,fifo_statu:6,file:[0,2,4,6,7,9],file_nam:[6,9],filenam:6,fill:[0,5,6],filter:6,filter_data:6,filter_downsampl:6,filter_select:6,filter_sum:6,find:9,finish:6,first:[1,5,6,7,9],fit:6,fix:6,flag:6,flat:6,flight:6,folder:8,follow:[1,2,7,8],fork:5,form:[6,9],format:[1,5,6],found:1,fpga:[0,2,6,8],fpga_bitfile_path:4,fpga_onli:8,fpga_test:[],fpga_xem7310:0,fpsdk:6,freq:6,frequenc:6,frequeni:[],from:[0,1,2,4,6,7],from_voltag:9,front:6,frontpanel:[1,4,6],frontpanel_path:4,frontpanelusb:4,fsdo:6,full:6,function_nam:6,futur:1,gain:6,gen_addr:1,gen_address:0,gen_bit:[0,1],gener:[0,1,6,9],get:[5,6,9],get_available_endpoint:[],get_chip_endpoint:[0,1],get_chip_regist:[0,7],get_device_cod:6,get_fifo_statu:6,get_gain:6,get_id:6,get_manufacturer_cod:6,get_master_configur:6,get_memory_usag:9,get_pll_statu:6,get_serial_numb:6,get_statu:6,get_tim:[],get_timing_pll_statu:6,github:[2,4,8],give:6,given:[0,4,6,7,9],glass:6,global:6,global_en:6,global_tim:6,go:6,go_bsi:6,goal:[],gp:1,gp_wire_in:1,great:5,greater:6,group:[0,1,6,7],guid:[3,4],h13:6,h3010:6,h5:9,h:1,ha:[0,6,7],half:6,handl:9,hardwar:6,have:[1,8],hdf:6,hdl:6,header:6,help:9,here:[1,7],hex:1,hexadecim:[1,7],high:[0,6,7],hold:[1,6,7],host:[1,6],how:0,html:[],http:6,hw_reset:6,hz:6,i2c:[5,6],i2c_configur:6,i2c_max_timeout_m:6,i2c_read_long:6,i2c_rec:6,i2c_transmit:6,i2c_write_long:6,i2ccontrol:[0,5],i2cdaq:0,i2cdaq_level_shift:0,i2cdaq_qw:0,i:6,id:[6,7],ie:6,ignor:[1,7],immedi:6,immeid:6,impact:[],imped:[2,6,9],impedance_analyz:2,impl_1:[],implement:0,improv:6,in_plac:[],includ:[1,2,5,6],incom:6,increment:[0,1,6],increment_endpoint:[],index:[0,1,3,5,6],indic:7,inform:[0,1,4,5],init_devic:0,initi:[0,6],inner:0,input:[6,9],instal:[3,8],installation_eripher:[],instanc:[0,5,6],instanti:[1,6],int_to_list:9,integ:[6,9],interact:6,interfac:[6,9],intern:[0,5,6,7],internship:0,interrupt:6,introduc:7,io:6,issu:5,its:[1,5,8],itself:0,june:[0,9],just:[1,6],k3:2,keep:5,kei:[1,6],kelli:[0,1,2,4,6],keyword:6,khz:6,know:[],known:2,koer2434:[0,9],koerner:[0,9],l3:2,label:7,last:[5,6],latch:6,later:6,latest:[],leav:[1,6],left:6,length:[0,6],level:0,light:6,like:[4,5,6],line:[1,6],link:5,list:[6,7,9],littl:9,load:0,locat:[0,6,7],lock:6,longer:6,loop:6,low:[0,6,7],lower:1,lpf:6,lsb:[0,6,9],lsbyte:6,luca:[0,9],lucask07:[],lvd:6,m:8,m_ndatastart:6,m_pbuf:6,macro:1,made:[],mai:[5,8,9],main:5,make:[],make_flat_voltag:6,make_ramp:6,make_sine_wav:6,make_step:6,mani:6,manufactur:6,march:[],margin:6,margin_high:6,margin_low:6,mark:8,markdown:[],marker:8,mask:[0,6],master:6,master_config:6,match:[6,7],matter:[1,7],maximum:6,mb:6,mean:9,mem:1,mem_wire_in:1,member:[],memori:[6,7],meth:[],method:[1,5,6,7,9],mhz:6,mig:6,millisecond:6,minim:6,minimum:9,mod:8,modifi:6,modul:[0,3,5,6,9],more:[1,5,6,8],msb:[0,6,9],msbyte:6,msg:6,msp:6,much:0,multipl:[0,6],multiplex:6,multipli:9,must:[0,1,6,7],mux:6,my_project:4,n2:2,n5:2,name:[0,1,5,6],ndarrai:[6,9],necessari:[6,9],need:[1,2,4,5,6],neg:6,new_frequ:6,newli:6,next:6,no_fpga:8,none:[6,9],note:[1,7],now:[0,6],np:6,num:9,num_bit:9,num_byt:[6,9],num_repeat:6,number:[6,9],number_of_byt:6,number_of_chip:6,numer:6,numpi:[5,6,9],numpydoc:[],o:6,object:[0,1,6,7,9],off:6,offset:6,often:1,ohm:6,ok:[0,6],okcfrontpanel:0,oktdeviceinfo:0,onc:[0,1,5],one:[6,7,8],onedr:0,onli:[0,1,6,7],opal:[0,1,2,4,6],opalkelli:[0,4,6],open:[4,5,7],oper:6,optic:6,option:[0,1,4,6],order:[1,6,7],org:[],organ:6,origin:6,oscilloscop:6,other:[1,2,4,6,7],otherwis:6,ouptut:6,our:7,out:6,outgo:6,output:6,over:[1,6],overwrit:9,own:1,p2:2,packag:4,package_nam:[],page:[0,3,5],pair:[0,1,6],paramet:[0,1,6,9],part:1,pass:6,path:[0,2,4],pattern:6,per:[6,9],percentag:6,perform:6,period:6,peripher:[0,2,3,7,8,9],phase:6,phrase:1,piec:1,pin:[2,6],pip:8,pipe:6,pipe_out:6,pipein:[],pipeout:[0,6],place:[0,6],pleas:5,pll:6,pm:[],point:6,pointer:6,posit:6,possibl:[1,6],power:6,power_down_10k:6,power_down_adc:6,power_down_al:6,power_down_fpga:6,power_down_high_imped:6,power_up:6,prb:6,preambl:6,preamble_length:6,prefix:[1,6,7],print:[0,6],print_fifo_statu:6,process:[5,6],program:[0,4],project:7,propos:[],protocol:6,provid:6,pull:[],puls:6,put:[1,6],py:[2,8],pypanel:[1,7],pyripher:[0,2,4,6,8,9],pytest:[5,8],python:[0,1,4],question:5,quickstart:[],qw:0,r:6,ramp:6,rang:[6,9],rate:6,rather:6,read:[0,2,5,6,7,9],read_adc:6,read_adc_block:6,read_channel:6,read_cnt:6,read_coeff_debug:6,read_data:6,read_en:[],read_ep:0,read_fg_en:[],read_h5:9,read_last:6,read_oper:6,read_out:7,read_pipe_out:0,read_trig:0,read_wir:0,read_wire_bit:0,readabl:7,readcheck:6,readi:6,readthedoc:[],reason:7,recal:1,receiv:6,recommend:[6,7],reduc:6,redund:7,ref:[],ref_pwdwn:6,refdiv:6,refer:[1,6],referenc:1,reflect:1,reformat:6,reg:0,reg_nam:6,reg_to_voltag:6,reg_val:6,reg_valu:6,regaddr:6,regist:[0,3,4,6],register_index_guide_completed_exampl:[],register_index_templ:[],register_nam:6,registerbridg:6,registers_path:4,releas:6,remain:5,reorder:6,repeat:[],repetit:6,replac:[1,7,8],repo:5,repositori:5,repres:[6,9],represent:[6,9],request:[],requir:[5,6,7,8],research:0,reset:6,reset_devic:6,reset_fifo:6,reset_mast:6,reset_mig_interfac:6,reset_pl:6,reset_trig:6,reset_wir:6,resist:9,resistor:[2,9],respect:[],rest:6,result:6,retriev:1,rev:6,revers:9,reverse_bit:9,right:6,rise:6,room:6,round:[6,9],rout:6,row:[1,7],run:[0,2,4,6,8],rx_neg:6,s:[0,1,2,6,7,9],same:[0,1,5,6],sample_s:6,sandbox:[],save:6,save_data:6,saw:6,sawtooth_fal:6,sawtooth_ris:6,scale:6,scl:6,sclk:6,script:5,sda:6,sdi:2,sdk:4,sdo:2,sdoa:2,sdob:2,search:3,section1:6,section:[1,4,6],see:[0,1,4,5,6,7],select:6,select_slav:6,send:[2,6],send_trig:0,sent:6,separ:[1,6],sequenc:6,seri:9,serial:6,set:[0,5,6],set_adc_dac_simultan:6,set_adc_debug:6,set_adc_read:6,set_adc_writ:6,set_adcs_connect:6,set_clk_divid:6,set_clk_rising_edg:6,set_config:6,set_config_bin:6,set_ctrl_reg:6,set_dac_read:6,set_dac_writ:6,set_data_mux:6,set_ddr_read:6,set_divid:6,set_en:6,set_endpoint:0,set_ep_simultan:0,set_fpga_mod:6,set_frequ:6,set_gain:6,set_gain_bin:6,set_host_mod:6,set_index:6,set_rang:6,set_read:[],set_spi_sclk_divid:6,set_wir:0,set_wire_bit:0,set_writ:[],setup:[2,6,8],setup_sequenc:6,sever:[5,6,9],shall:6,sheet:[0,7],shell:4,shift:[0,6],shorten:1,should:[1,5,6,7],show:5,shown:4,side:[],sign:6,signal:6,similar:[7,9],simpli:8,sinc:[1,6,7],sine:6,singl:[0,6],size:[6,9],skip:6,slave:6,slave_address:6,slew_rat:6,slope:6,smudg:6,so:[6,7],soft:6,softwar:6,sort:9,sourc:[0,6,9],sources_1:0,space:6,specif:[0,5,6,8],speed:6,speed_mb:6,spi:[5,6],spi_control:6,spi_fifo_driven:6,spicontrol:[0,5],spififodriven:[0,5],split:1,spreadsheet:[0,1,4,5],squar:6,src:0,st:0,stack:6,standard:[],start:[0,1,6,7],start_func:6,startup:6,statu:6,step:[5,6,7],stop:6,stop_func:6,store:[0,1,6,7],str:[0,6,9],stream:6,string:[6,7],stripe:6,stroschein:[0,9],structur:1,stthoma:[0,9],subclass:6,subpackag:[5,6],suffici:6,suffix:1,suggest:6,sum:6,support:[4,6],sure:[5,7],swap:6,synchron:6,t0:[],t1:[],t:6,take:6,taken:7,target:6,tca9555:6,tclk:6,tell:1,templat:[1,7],test:[2,3,6],test_pattern:6,text:0,than:[1,6],thei:[1,6,7],themselv:1,thi:[0,1,2,5,6,7],thoma:0,through:[1,2,5,6],througput:6,time:[6,9],timeout:6,timestamp:6,titl:7,tmf8801:6,to_voltag:9,todo:6,togeth:[],toggl:0,toggle_high:0,toggle_low:0,tooth:6,top_level_modul:[2,4],tos:6,total:[6,9],track:[],transfer:6,transmiss:6,transmit:6,tree:[],triangl:6,trigger:[0,6],triggerin:[0,6],triggerout:0,tupdat:6,turn:6,two:[1,6],twos_comp:9,tx_neg:6,type:6,uid_24aa025uid:6,uint32:6,unaffect:6,under:1,underscor:[1,7],understand:2,uniqu:6,unit16:6,univers:0,unknown:[2,9],unlock:6,unsuccess:6,unsupport:4,until:6,up:[5,6],updat:[4,6],update_endpoints_from_defin:0,upper:[],uppercas:7,us:[0,1,2,4,5,6,7,8,9],usabl:8,usag:0,usb3:6,usb:1,use_twos_comp:9,user:[0,4,6],usernam:4,util:[3,4],v:[0,1,4,6],v_in:9,v_out:9,val:[6,9],val_list:0,valid:7,valu:[0,1,2,6,9],veri:[],verilog:[0,1,6,7],version:[6,9],via:6,voltag:[2,6,9],voltage_rang:[6,9],vref:6,w0:[],w127:[],w128:[],w15:[],w224:[],w255:[],w31:[],w32:[],w63:[],w:6,wa:[6,9],wai:[5,6],wait:6,want:5,watch:5,wave:6,waveform:6,wb_clk_freq:6,wb_go:6,wb_is_ack:6,wb_read:6,wb_send_cmd:6,wb_set_address:6,wb_write:6,wbsignal_convert:6,we:[6,7,9],well:[1,2,4,6],what:[5,6],whatev:[1,7],when:[0,1,6],where:6,whether:[0,6],which:[1,2,7],wide:[],width:[0,1,9],wire:[0,6],wire_in:1,wire_out:6,wirein:[0,6],wireout:0,wishbon:6,with_neg:9,within:[6,7],word:[6,7],word_address:6,words_read:6,work:[5,8],workbook_path:0,would:1,wrap:6,write:[5,6],write_channel:6,write_chip_reg:6,write_en:[],write_in:7,write_oper:6,write_reg_bridg:6,write_voltag:6,written:[0,1,5,6],x1:6,x2:6,x:6,xem7310:[0,2],xem:0,xlsx:[0,4],yaml:9,yet:0,you:[1,2,4,7,8],your:[1,2,5,7],zero:[6,7]},titles:["core","Endpoint Definitions Guide","Example","Welcome to pyripheral\u2019s documentation!","Installation","New Peripheral Guide","peripherals","Register Index Guide","Tests","utils"],titleterms:{"1":7,"2":7,"3":7,"4":7,"5":7,"6":7,"class":[],"default":7,"new":5,address:[1,7],base:6,befor:5,begin:5,bit:[1,7],board:[],chip:7,configur:4,content:3,control:6,core:0,covgdaq:[],ddr:[],definit:1,document:[3,5],driven:[],endpoint:1,enter:7,exampl:2,extend:6,fifo:[],file:[1,5],fpga:4,guid:[1,5,7],hex:7,i2c:[],i2ccontrol:6,index:7,indic:[1,3],instal:4,interfac:5,lower:7,memori:[],miscellan:6,modul:[],name:7,peripher:[4,5,6],pip:4,pull:5,py:[],pyripher:3,quick:4,regist:[5,7],remain:7,repeat:7,request:5,s:3,sandbox:5,spi:[],spicontrol:6,spififodriven:6,start:4,tabl:3,test:[5,8],upper:7,usag:1,util:9,valu:7,welcom:3,width:7,yaml:4,you:5}})