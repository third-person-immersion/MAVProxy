#!/usr/bin/env python
'''tune command handling'''

import time, os
from MAVProxy.modules.lib import mp_module

class SpeechModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SpeechModule, self).__init__(mpstate, "speech", "speech output")
        self.mpstate.functions.say = self.say
        self.settings.append(('speech', int, 1))
        self.kill_speech_dispatcher()
        self.say('')

    def kill_speech_dispatcher(self):
        '''kill speech dispatcher processs'''
        pidpath = os.path.join(os.environ['HOME'], '.speech-dispatcher',
                               'pid', 'speech-dispatcher.pid')
        if os.path.exists(pidpath):
            try:
                import signal
                pid = int(open(pidpath).read())
                if pid > 1 and os.kill(pid, 0) is None:
                    print("Killing speech dispatcher pid %u" % pid)
                    os.kill(pid, signal.SIGINT)
                    time.sleep(1)
            except Exception as e:
                pass


    def unload(self):
        '''unload module'''
        self.kill_speech_dispatcher()

    def say(self, text, priority='important'):
        '''speak some text'''
        ''' http://cvs.freebsoft.org/doc/speechd/ssip.html see 4.3.1 for priorities'''
        self.console.writeln(text)
        if self.settings.speech:
            import speechd
            self.speech = speechd.SSIPClient('MAVProxy%u' % os.getpid())
            self.speech.set_output_module('festival')
            self.speech.set_language('en')
            self.speech.set_priority(priority)
            self.speech.set_punctuation(speechd.PunctuationMode.SOME)
            self.speech.speak(text)
            self.speech.close()

def init(mpstate):
    '''initialise module'''
    return SpeechModule(mpstate)
