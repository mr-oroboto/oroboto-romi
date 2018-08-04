#include "sound.h"

#define FREQUENCY_MODE_ABORTED_WAYPOINT 800
#define FREQUENCY_MODE_ALARM 200

//Sound::Sound()
//{
//}

void Sound::finished()
{
    buzzer.playFromProgramSpace(soundFinished);
    delay(2000);  
}

void Sound::ok()
{
    buzzer.playFromProgramSpace(soundOk);
    delay(500);  
}

void Sound::alarm(bool wait)
{
    playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);

    if (wait)
    {
        delay(FREQUENCY_DURATION);
    }
}

void Sound::abortedWaypoint()
{
    playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
    delay(FREQUENCY_DURATION);
}

void Sound::playFrequency(int mode, int duration, int volume)
{
    buzzer.playFrequency(mode, duration, volume);
}

