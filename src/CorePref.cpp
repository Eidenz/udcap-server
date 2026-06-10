//
// Created by max_3 on 25-6-20.
//

#include "CorePref.h"

void CorePref::setDefaultPrefPath(std::string path) {
    this->prefPath = path;
}

std::filesystem::path CorePref::getPrefPath() const {
    return this->prefPath;
}

CorePref::CorePref() {
    // Default to a "prefs" directory under the current working directory.
    // Callers (e.g. the Monado driver) should override via setDefaultPrefPath()
    // with an absolute, app-owned location.
    this->prefPath = std::filesystem::path("prefs");
}

CorePref& CorePref::getInstance() {
    static CorePref instance;
    return instance;
}