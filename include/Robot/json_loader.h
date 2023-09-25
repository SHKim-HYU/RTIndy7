/*! 
 *  @file json_loader.h
 *  @brief header for loading json file
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Sep. 22. 2023
 *  @Comm
 */

#ifndef JSON_LOADER_H
#define JSON_LOADER_H

#include <iostream>
#include <fstream>
#include <json/json.h>

class JsonLoader {
private:
    Json::Value data;

public:
    // Constructor: Takes the path of the JSON file as an argument.
    JsonLoader(const std::string &filePath) {
        std::ifstream input_file(filePath);
        if (!input_file.is_open()) {
            std::cerr << "Failed to open JSON file." << std::endl;
            return;
        }

        Json::CharReaderBuilder rbuilder;
        std::string errs;
        bool parsingSuccessful = Json::parseFromStream(rbuilder, input_file, &data, &errs);
        if (!parsingSuccessful) {
            std::cerr << "Failed to parse JSON: " << errs << std::endl;
            return;
        }

        input_file.close();
    }

    // Function to fetch the value corresponding to a specific key in the JSON object.
    Json::Value getValue(const std::string &key) const {
        if (data.isMember(key)) {
            return data[key];
        }
        return Json::Value(); 
    }
};

#endif  // JSON_LOADER_H