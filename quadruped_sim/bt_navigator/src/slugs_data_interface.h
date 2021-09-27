#pragma once
#include <fstream>
#include <iostream>
#include <list>
#include <iterator>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

class SlugsDataInterface
{
  public:
    SlugsDataInterface(json root);

    //Setters - variables that can be changed from env
    bool set_env_var(bool r1ml, 
                    bool r2ml, 
                    int bl, 
                    bool np);
    // bool change_rm_1_meet_locked,
    // bool change_rm_2_meet_locked, 
    // bool change_battery_level, 
    // bool change_near_ppl,   

    //TODO: 
    // Add in all environment variables including time

  private:
   
    //Private nested class only accessible by SlugsDataInteface
    class SlugsData 
    {
      public: 
        SlugsData(){ /* empty constructor */ }

        bool compare(SlugsData s){
            return (time == s.time &&
                    rm_1_cleaned == s.rm_1_cleaned && 
                    rm_2_cleaned == s.rm_2_cleaned &&
                    in_rm == s.in_rm && 
                    loc_st == s.loc_st && 
                    rm_1_sche_locked == s.rm_1_sche_locked && 
                    rm_2_sche_locked == s.rm_2_sche_locked && 
                    rm_1_meet_locked == s.rm_1_meet_locked && 
                    rm_2_meet_locked == s.rm_2_meet_locked && 
                    battery_level == s.battery_level && 
                    near_ppl == s.near_ppl && 
                    task_cmplt == s.task_cmplt && 
                    action == s.action && 
                    loc_action == s.loc_action);
        }          
        
        //Variables are public temporarily
        //Annoying to make another set of 
        // getter and setter functions            
        long state_num = 0;

        //These values are specified in slugs file
        int time = 30; 
        bool rm_1_cleaned = false;
        bool rm_2_cleaned = false;
        int in_rm = 0;
        int loc_st = 3;
        bool rm_1_sche_locked = true;
        bool rm_2_sche_locked = true;
        bool rm_1_meet_locked = false;
        bool rm_2_meet_locked = false;
        int battery_level = 10;
        bool near_ppl = false;
        bool task_cmplt = false;
        int action = 0;
        int loc_action = 3;
    };

    //Variables
    json root_;
    SlugsData data_;        
    std::list<long> transitions_;
    int state_size_;  

    void set_new_state(long temp_state_num);

    bool search_transition(SlugsData temp, long& temp_state_num);

    SlugsData arr_to_state(int *arr);

  public:
    int get_state_size(){
        return state_size_;
    }

    //Getters
    int get_time(){
        return data_.time;
    }
    bool get_rm_1_cleaned(){
        return data_.rm_1_cleaned;
    }
    bool get_rm_2_cleaned(){
        return data_.rm_2_cleaned;
    }
    int get_in_rm(){
        return data_.in_rm;
    }
    int get_loc_st(){
        return data_.loc_st;
    }
    bool get_rm_1_sche_locked(){
        return data_.rm_1_sche_locked;
    }
    bool get_rm_2_sche_locked(){
        return data_.rm_2_sche_locked;
    }
    bool get_rm_1_meet_locked(){
        return data_.rm_1_meet_locked;
    }
    bool get_rm_2_meet_locked(){
        return data_.rm_2_meet_locked;
    }   
    int get_battery_level(){
        return data_.battery_level;
    }
    bool get_near_ppl(){
        return data_.near_ppl;
    }
    bool get_task_cmplt(){
        return data_.task_cmplt;
    }
    int get_action(){
        return data_.action;
    }
    int get_loc_action(){
        return data_.loc_action;
    }
};



