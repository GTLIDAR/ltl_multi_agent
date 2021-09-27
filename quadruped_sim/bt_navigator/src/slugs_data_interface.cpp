#include "slugs_data_interface.h"


using json = nlohmann::json;


    SlugsDataInterface::SlugsDataInterface(json root) : root_(root)
    { 
        state_size_ = root_["nodes"]["0"]["state"].size();

        SlugsData temp;
        data_ = temp;

        //Use the "new" keyword to ensure data_ doesn't reset
        //data_ = new SlugsData();
        std::cout << "constructor" << std::endl;
        for (auto& longs : root_["nodes"]["0"]["trans"])
            transitions_.push_back(longs);
    }
    
    //Setters - variables that can be changed from env
    bool SlugsDataInterface::set_env_var(    
        bool r1ml, 
        bool r2ml, 
        int bl, 
        bool np)
    {
        // bool change_rm_1_meet_locked,
        // bool change_rm_2_meet_locked, 
        // bool change_battery_level, 
        // bool change_near_ppl,    

        // //Zeroth step is to check if it has any changes
        // if(!(change_rm_1_meet_locked ||
        //     change_rm_1_meet_locked ||
        //     change_battery_level ||
        //     change_near_ppl)){
        //     std::cout << "Tried to change environment variable" << 
        //                     "without any changes?" << std::endl;
        //     return true;
        // }
        std::cout << "Setting env var " << std::endl;


        //First step is to copy the current state
        SlugsData temp = data_;

        //Then apply the adjustments 
        // if(change_rm_1_meet_locked){
            temp.rm_1_meet_locked = r1ml;
        // }
        // if(change_rm_2_meet_locked){
            temp.rm_2_meet_locked = r2ml;
        // }
        // if(change_battery_level){
            // temp.battery_level = bl;
        // }
        // if(change_near_ppl){
            temp.near_ppl = np;
        // }

        //Lastly, validate transition and modify data
        long temp_state_num = 0;
        if(search_transition(temp, temp_state_num)){
            std::cout << "received new transition" << std::endl;
            set_new_state(temp_state_num);
            return true;
        } else {
            std::cout << "Failed to set new transition" << std::endl;
            return false;
        }
    }

    void SlugsDataInterface::set_new_state(long temp_state_num)
    {
        data_.state_num = temp_state_num;
        int arr[state_size_];
        for(int i = 0; i < state_size_; i++){
            arr[i] = root_["nodes"][std::to_string(temp_state_num)]["state"][i];
        }

        //Call function to set data_ to the new array
        data_ = arr_to_state(arr);
    }

    bool SlugsDataInterface::search_transition(SlugsData temp, 
                                                long& temp_state_num)
    {

        std::cout << "searching transition" << std::endl;

        //Loop through all transitions and find a matching state
        std::list <long> :: iterator it;
        for(it = transitions_.begin(); it != transitions_.end(); ++it){

            //Get the transitions's state in array form
            int arr [state_size_];

            for(int i = 0; i < state_size_; i++){
                arr[i] = root_["nodes"][std::to_string(*it)]["state"][i];
            }
            //Compare if candidate transition is one of them
            if(temp.compare(arr_to_state(arr))){
                temp_state_num = *it; 
                return true;
            }
        }
        return false;
    }

    SlugsDataInterface::SlugsData SlugsDataInterface::arr_to_state(int *arr)
    {
        SlugsData s;
        std::cout << "arr to state" << std::endl;
        std::string temp = "";
        for(int i = 7; i >= 0; i--){
            temp += std::to_string(arr[i]);
        }
        s.time = std::stoi(temp, nullptr, 2);
        s.rm_1_cleaned = arr[8];
        s.rm_2_cleaned = arr[9];
        temp = ""; //clear string
        temp += std::to_string(arr[11]) + std::to_string(arr[10]); //Must be concatenated backwards
        s.in_rm = std::stoi(temp, nullptr, 2);
        temp = ""; //clear string
        temp += std::to_string(arr[13]) + std::to_string(arr[12]); //Must be concatenated backwards
        s.loc_st = std::stoi(temp, nullptr, 2);
        s.rm_1_sche_locked = arr[14];
        s.rm_2_sche_locked = arr[15];
        s.rm_1_meet_locked = arr[16];
        s.rm_2_meet_locked = arr[17];
        temp = ""; //clear string
        for(int i = 21; i >= 18; i--){
            temp += std::to_string(arr[i]);
        }
        s.battery_level = std::stoi(temp, nullptr, 2);
        s.near_ppl = arr[22];
        s.task_cmplt = arr[23];
        temp = ""; //clear string
        for(int i = 26; i >= 24; i--){
            temp += std::to_string(arr[i]);
        }
        s.action = std::stoi(temp, nullptr, 2);
        temp = ""; //clear string
        temp += std::to_string(arr[28]) + std::to_string(arr[27]); //Must be concatenated backwards
        s.loc_action = std::stoi(temp, nullptr, 2);

        //Delete current instance of data_
        //delete data_;

        //Set new data_ to new instance
        //data_ = s;
        
        return s;
    }





