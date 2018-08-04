#ifndef __SOUND_H__
#define __SOUND_H__

#include <Romi32U4.h>

#define FREQUENCY_DURATION 300
#define FREQUENCY_VOLUME 9

class Sound
{
  public:
//    Sound();  

    void finished();
    void ok();
    void alarm(bool wait);
    void abortedWaypoint();

  private:
    void playFrequency(int mode, int duration, int volume);

    Romi32U4Buzzer buzzer;

    const char soundFinished[] PROGMEM = "! L16 V8 cdefgab>cbagfedc";
    const char soundOk[] PROGMEM = "v10>>g16>>>c16";
};

#endif // __SOUND_H__
