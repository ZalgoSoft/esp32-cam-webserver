#include "FS.h"
#include "SPIFFS.h"

#define FORMAT_SPIFFS_IF_FAILED true
#define PREFERENCES_MAX_SIZE 500

#define PREFERENCES_FILE "/esp32cam-preferences.json"
#define PREFERENCES_POS_FILE "/ptz-last-pos.json"

extern void dumpPrefs(fs::FS &fs);
extern void loadPrefs(fs::FS &fs);
extern void removePrefs(fs::FS &fs);
extern void savePrefs(fs::FS &fs);
extern void loadposPrefs(fs::FS &fs);
extern void removeposPrefs(fs::FS &fs);
extern void saveposPrefs(fs::FS &fs);

extern void filesystemStart();
