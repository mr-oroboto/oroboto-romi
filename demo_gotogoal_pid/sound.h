#ifndef __SOUND_H__
#define __SOUND_H__

#include <Romi32U4.h>

#define FREQUENCY_DURATION 300
#define FREQUENCY_VOLUME 9

class Sound
{
  public:
    void finished();
    void ok();
    void alarm(bool wait);
    void abortedWaypoint();

  private:
    void playFrequency(int mode, int duration, int volume);

    Romi32U4Buzzer buzzer;
};

#endif // __SOUND_H__
