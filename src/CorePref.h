//
// Created by max_3 on 25-6-20.
//

#ifndef UDCAPCOMMUNITYDRIVERCORETEST_COREPREF_H
#define UDCAPCOMMUNITYDRIVERCORETEST_COREPREF_H

#include <string>
#include <filesystem>
class CorePref {
public:
    void setDefaultPrefPath(std::string path);
    static CorePref& getInstance();
    std::filesystem::path getPrefPath() const;
    // Deleted to enforce singleton
    CorePref(const CorePref&) = delete;
    CorePref& operator=(const CorePref&) = delete;
private:
    CorePref();
    ~CorePref() = default;
    std::filesystem::path prefPath;
};


#endif //UDCAPCOMMUNITYDRIVERCORETEST_COREPREF_H
