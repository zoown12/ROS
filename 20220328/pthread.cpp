#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
                   void *(*start_routine)(void *), void *arg);
/* 
    thread 생성을 위해서 사용.
    첫번째 아규먼트인  thread 는 thread가 성공적으로 생성되었을때 thread를 식별하기 위해서 사용되는 쓰레드 식별자.
    두번째 아규먼트인 attr은 thread 특성을 지정하기 위해서 사용, 기본 thread 특성을 이용하고자 할 경우 NULL 사용.
    세번째 아규먼트인 start_routine는 분기시켜서 실행할 thread 함수.
    네번째 아규먼트인 arg는 thread 함수의 인자.
    성공적으로 생성될 경우 0 리턴.
    
    실행된 thread에 대해서는 pthread_join 등의 함수를 이용해서 thread 종료때까지 기다려줘야함. 
    pthread_join 은 일종의 fork 의 wait 와 비슷하게 작동. thread의 자원을 해제.

*/

int pthread_join(pthread_t th, void **thread_return);
 
/*
     첫번째 아규먼트 th는 기다려야할 thread 식별자.
     두번째 아규먼트 thread_return은 thread의 리턴 값.
     thread_return 이 NULL 이 아닐경우 해다 포인터로 thread 리턴 값을 받아올수 있다.  
*/

int pthread_detach(pthread_t th);
/*
     main thread 에서 pthread_create 를 이용해 생성된 thread를 분리. 
     이 함수는 식별번호 th인 thread를 detach 시키는데, detach 되었을경우 해당 thread가 종료될 경우 pthread_join 을 호출하지 않더라고
     즉시 모든 자원이 해제.
*/

void pthread_exit(void *retval);
/*
    pthread_exit 는 현재 실행중인 thread를 종료시키고자 할때 사용한다. 
    만약 pthread_cleanup_push 가 정의되어 있다면, pthread_exit 가 호출될 경우 cleanup handler 가 호출됨.
    cleanup handler 은 메모리 정리하는 등의 일을 하게 된다.
*/

void pthread_cleanup_push(void (*routine) (void *), void *arg);

/*
    cleanup handlers 를 인스톨하기 위해서 사용. pthread_exit(3)이 호출되어서 thread가 종료 될때 pthread_cleanup_push에 의해서 인스톨된 함수가 호출됨.
    routine이 thread가 종료될때 호출되는 함수. arg는 아규먼트. 
    cleanup handlers 는 주로 자원을 되돌려주거나, mutex 잠금등의 해제를 위한 용도로 사용. mutex 영역에서 pthread_exit가 호출되면 다른thread에서 영원히 갇힐수 있다.
    malloc 으로 할당받은 메모리, 열린 파일지정자를 닫기 위해서 사용.

    
*/

void pthread_cleanup_pop
/*
   0일 경우:pthread_cleanup_push 와 함께 사용되며, install 된 cleanup handler를 삭제시킴. 
   0일 아닐 경우: cleanup handler을 실행시키고 삭제 됨. 
    pthread_cleanup_push 와 pthread_cleanup_pop 은 반드시 같은 함수내의 같은 레벨의 블럭에서 한쌍으로 사용해야 함.
*/

pthread_t pthread_self(void);
/*
    pthread_self를 호출하는 현재 thread의 thread식별자를 되돌려줌.
*/

int pthread_mutex_init(pthread_mutex_t * mutex, 
           const pthread_mutex_attr *attr); 
/*
   mutex는 여러개 thread가 공유하는 데이터를 보호하기 위해 사용되는 도구. 보호하고자 하는 데이타를 다루는 코드영역을 단지 한번에 하나의 thread만 실행 가능 하도록 하는 방법으로 공유되는 데이타를 보호한다.
   이러한 코드영역을 critical section 이라고 하며, mutex 관련 API를 이용해서 관리한다. 
   pthread_mutex_init 는 mutex 객체를 초기화 시키기 위해서 사용. 
   첫번째 인자로 주어지는 mutex 객체 mutex를 초기화 시키며, 두번째 인자인 attr 를 이용해서 mutex 특성을 변경.
   기본 mutex 특성을 이용하기 원한다면 NULL 사용. 
   mutex 종류: fast,recurisev,error checking 기본적으로 fast 사용.

*/

int pthread_mutex_destroy(pthread_mutex_t *mutex);
/*
   인자로 주어진 mutex 객체 mutex를 제거하기 위해서 사용. 
   pthread_mutex_destory 를 이용해서 제대로 mutex 를 삭제하려면 이 mutex는 반드시 unlock 상태이여야 한다.
*/


int pthread_mutex_lock(pthread_mutex_t *mutex);
/*
   pthread_mutex_lock 는 critical section 에 들어가기 위해서 mutex lock을 요청한다. 만약 이미 다른 thread 에서 mutex lock를 얻어서 사용하고 있다면 다른 thread 에서 mutex lock을 해체할때까지 block 됨.
   다른 thread 에서 사용을 하지 않고 있다면, 즉시 mutex lock 을 얻을수 있게 되고 critical section 에 진입하게 된다. 
   critical section 에서의 모든 작업을 마쳐서 사용하고 있는  mutex lock 이 더이상 필요 없다면 pthread_mutex_unlock 를 호출해서 mutex lock를 되돌려준다.
*/

int pthread_mutex_unlock(pthread_mutex_t *mutex); 
/*
    critical section 에서의 모든 작업을 마치거 mutex lock 을 돌려주기 위해서 사용. 
    pthread_mutex_unlock 를 이용해서 mutex lock 를 되돌려주면 다른 thread 에서 mutex lock 를 얻을수 있는 상태가 된다.
*/

int pthread_cond_init(pthread_cond_t *cond, 
                    const pthread_cond_attr *attr);
/*
   pthread_cond_init는 조견변수 (condition variable)cond를 초기화하기 위해서 사용한다. attr 를 이용해서 조건변수의 특성을 변경할수 있으며, NULL 을 줄경우 기본특성으로 초기화된다.

   조건변수 cond는 상수 PTHREAD_COND_INITIALIZER 을 이용해서도 초기화 할수 있다. 즉 다음과 같은 2가지 초기화 방법이 존재한다.

    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
  
    pthread_cond_init(&cond, NULL);
*/

int pthread_cond_signal(pthread_cond_t *cond);
/*
    조건변수 cond에 시그날을 보낸다. 시그날을 보낼경우 cond 에서 기다리는 thread가 있다면 thread를 깨우게 된다.
    만약 cond를 기다리는 thread가 없다면, 아무런 일도 일어나지 않게되며, 여러개의 thread가 기다리고 있다면 그중 하나의 thread에게만 전달.
    어떤 thread에 신호가 전될되는지는 모름.
*/
int pthread_cond_broadcast(pthread_cond_t *cond);
/*
조건변수 cond에서 기다리는(wait) 모든 쓰레드에게 신호를 보내서, 깨운다는 점을 제외하고는 pthread_cond_signal과 동일하게 작동한다.
*/

int pthread_cond_wait(pthread_cond_t cond, pthread_mutex_t *mutex); 
/*    
조건변수 cond를 통해서 신호가 전달될때까지 블럭된다. 
만약 신호가 전달되지 않는다면 영원히 블럭될수도 있다. pthread_cond_wait는 블럭되기 전에 mutex 잠금을 자동으로 되돌려준다.
*/

int pthread_cond_timedwait(pthread_cont_t *cond, pthread_mutex_t *mutex, 
                           const struct timespec *abstime);
 /*
    조건변수 cond를 통해서 신호가 전될될때까지 블럭되며 자동으로 mutex을 돌려주는 점에서는 pthread_cond_wait와 동일하다.
    그러나 시간체크가 가능해서 abstime시간동안 신호가 도착하지 않는다면 error를 발생하면서 리턴한다. 이때 리턴값은 ETIMEDOUT 이다.
    error가 세팅되는게 아닌, 리턴값으로 에로가 넘어오는것에 주의해야 된다. 
    pthread_cond_timewait 함수는 다른 signal 에 의해서 방해 될수 있으며 이때 EINTR 을 리턴.
*/

int pthread_cond_destroy(pthread_cond_t *cond);
/*
    pthread_cond_init를 통해서 생성한 조건변수 cond에 대한 자원을 해제한다. destory 함수를 호출하기 전에 어떤 thread도 cond에서의
    시그널을 기다리지 않는걸 확인해야 한다. 만약 cond 시그널을 기다리는 thread 가 존재한다면 이 함수는 실패하고 EBUSY를 리턴.
*/

int pthread_attr_init(pthread_attr_t *attr);

/*    
pthread_attr_init는 thread attribute 객체인 attr을 디폴트 값으로 초기화 시킨다.

성공할경우 0을 돌려주고 실패할경우 -1 을 되돌려준다.
*/


int pthread_attr_destroy(pthread_attr_t *attr);
/*    
pthread_attr_init에 의해 생성된 thread attribute 객체인 attr을 제거한다. 
제거된 attr 을 다시 사용하기 위해서는 pthread_attr_init를 이용해서 다시 초기화 해주어야 한다.
*/

int pthread_attr_getscope(const pthread_attr_t *attr, 
             int *scope);
 /*
   thread가 어떤 영역에서 다루어지고 있는지를 얻어오기 위해서 사용. 
   PTHREAD_SCOPE_SYSTEM과 PTHREAD_SCOPE_PROCESS 의 2가지 영역중에 선택.
   SYSTEM 영역 thread는 user 모드 thread라 불리우며, PROCESS thread는 커널모드 thread 라 불린다.
   리눅스의 경우 thread를 스케쥴링하는 방식이 아닌 thread 라이브러리를 통해서 thread를 스케쥴링 하는 방식.
*/   

 int pthread_attr_setscope(pthread_attr_t *attr, int scope);
/*
  thread가 어떤 영역에서 작동하게 할것인지 결정하기 위해서 사용. 리눅스의 경우 Kernel mode thread를 지원하지 않음.
  오직 PTHREAD_SCOPE_SYSTEM 만을 선택. 
*/

int pthread_attr_getdetachstate(pthread_attr_t *attr,
           int detachstate);
/*
   thread가 join 가능한 상태 인지 detached 상태인지 알아낸다.
   알아낸 값은 아규먼트 detachstate에 저장.
   기본은 PTHREAD_CREATE_JOINABLE 이며, pthread_detach를 이용해서 생성된 쓰레드를 detach 상태로 만들었을경우 
   또는 pthread_attr_setdetachstate함수를
   이용해서 쓰레드를 detache 상태로 변경시켰을경우 PTHREAD_CREATE_DETACHED 상태가 된다.
*/
int  pthread_attr_setdetachstate(pthread_attr_t *attr, 
             int detachstate);
/*
쓰레드의 상태를 PTHREAD_CREATE_JOINABLE 혹은 PTHREAD_CREATE_DETACHED 상태로 변경시키기 위해서 사용된다. 
아래와 같은 방법으로 사용하면 된다.

pthread_attr_t attr;
// JOINABLE 상태로 변경하고자 할때 
pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
// DETACHED 상태로 변경하고자 할때
pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
*/

int pthread_sigmask(int how, const sigset_t *newmask, sigset_t *oldmask);
/*
쓰레드에서 시그널은 서로 공유된다. 그런이유로 만약 프로세스에 시그널이 전달되면 프로세스가 생성된 모든 쓰레드로 시그널이 전달된다. 
그러나 특정 쓰레드만 시그널을 받도록 하고 싶을 때가 있을 것이다. 이경우 이 함수를 이용하면 된다.
*/

int pthread_kill(pthread_t thread, int signo);
/*
쓰레드 식별번호 thread로 signo번호의 시그널을 전달한다.
*/

int sigwait(const sigset_t *set, int *sig);
 /* 
시그널 전달을 동기적으로 기다린다.
*/

/* THREAD CANCEL */
int pthread_cancel(pthread_t thread);
   
int pthread_setcancelstate(int state, int *oldstate);
   
int pthread_setcancelstate(int state, int *oldstate);
   
int pthread_setcanceltype(int type, int *oldtype);

void pthread_testcancel(void);
 