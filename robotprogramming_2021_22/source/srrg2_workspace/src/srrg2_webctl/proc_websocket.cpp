#include <unistd.h>
#include <iostream>
#include <fstream>
#include <map>
#include <list>
#include <string>
#include <signal.h>
#include <sstream>
#include <libwebsockets.h>
#include <set>
#include "proc_shell.h"

#define BUF_SIZE 1024*1024
#define MAX_CONNECTIONS 1024
#define PATH_SIZE 1024

using namespace std;

typedef struct lws * WebSocketConnectionPtr;
struct WebSocketServer {
  char  _client_response[BUF_SIZE];
  char* _client_response_begin;
  int   _client_response_length=0;
  std::set<WebSocketConnectionPtr> _connections;
  pthread_t _thread;
  volatile int _run=false;
  int _port=9000;
  char _resource_path[PATH_SIZE];
  struct lws_context *_context=0;
  WebSocketServer();
  void start(int port, const char* resource_path);
  void stop();
  void initProtocols();
  static int callback_http(struct lws *wsi,
                           enum lws_callback_reasons reason,
                           void *user,
                           void *in, size_t len);
  static int callback_input_command( /*struct libwebsocket_context * this_context,*/
                                  struct lws *wsi,
                                  enum lws_callback_reasons reason,
                                  void *user, void *in, size_t len);
  struct lws_protocols _protocols[3];
  static void* _websocketThread(void* args);
  struct lws_context_creation_info _info;

};

static struct lws_http_mount mount = {
        /* .mount_next */		NULL,		/* linked-list "next" */
        /* .mountpoint */		"/",		/* mountpoint URL */
        /* .origin */			NULL, /* serve from dir */
        /* .def */			"index.html",	/* default filename */
        /* .protocol */			NULL,
        /* .cgienv */			NULL,
        /* .extra_mimetypes */		NULL,
        /* .interpret */		NULL,
        /* .cgi_timeout */		0,
        /* .cache_max_age */		0,
        /* .auth_mask */		0,
        /* .cache_reusable */		0,
        /* .cache_revalidate */		0,
        /* .cache_intermediaries */	0,
        /* .origin_protocol */		LWSMPRO_FILE,	/* files in a dir */
        /* .mountpoint_len */		1,		/* char count */
        /* .basic_auth_login_file */	NULL,
};

WebSocketServer srv;

WebSocketServer::WebSocketServer() {
   _client_response_begin=_client_response + LWS_PRE;
}

void WebSocketServer::initProtocols() {
  _protocols[0].name     = "http-only";
  _protocols[0].callback = lws_callback_http_dummy;
  _protocols[0].per_session_data_size = 0;
  _protocols[0].rx_buffer_size = 0;
  _protocols[0].user = this;


  _protocols[1].name     = "srrg-webcontrol-protocol";
  _protocols[1].callback = callback_input_command;
  _protocols[1].per_session_data_size = 0;
  _protocols[1].rx_buffer_size = 0;
  _protocols[1].user = this;

  _protocols[2].name     = NULL;
  _protocols[2].callback = NULL;
  _protocols[2].per_session_data_size = 0;
  _protocols[2].rx_buffer_size = 0;
  _protocols[2].user = this;  // does not work
}

void WebSocketServer::start(int port, const char* resource_path) {
  if (_run)
    return;
  
  const char *interface = NULL;
  initProtocols();
  _port=port;
  strcpy(_resource_path, resource_path);
  mount.origin=_resource_path;
  memset(&_info, 0, sizeof (_info));
  _info.port = _port;
  _info.iface = interface;
  _info.protocols = _protocols;
  _info.ssl_cert_filepath = NULL;
  _info.ssl_private_key_filepath = NULL;
  _info.gid = -1;
  _info.uid = -1;
  _info.mounts=&mount;
  cerr << "all fine so far" << endl;
  _context = lws_create_context(&_info);    
  if (_context == NULL) {
    cerr << "libwebsocket init failed, no websocket server active" << endl;
  }
  cerr << "done" << endl;
  _run=1;

  pthread_attr_t attr;
  pthread_attr_init(&attr);
  cerr << "starting thread" << endl;
  pthread_create(&_thread, &attr, _websocketThread, this);
}

void WebSocketServer::stop() {
  if (!_run)
    return;
  _run=0;
  void* retval;
  pthread_join(_thread, &retval);
  //free(context);
}


int WebSocketServer::callback_input_command( /*struct libwebsocket_context * this_context,*/
                                  struct lws *wsi,
                                  enum lws_callback_reasons reason,
                                  void *user, void *in, size_t len) {
  WebSocketServer& ws_ctx=srv;
  switch (reason) {
  case LWS_CALLBACK_ESTABLISHED: // just log message that someone is connecting
    printf("connection established\n");
    ws_ctx._connections.insert(wsi);

  case LWS_CALLBACK_SERVER_WRITEABLE: // just log message that someone is connecting
    lws_write(wsi, (unsigned char*) ws_ctx._client_response, ws_ctx._client_response_length, LWS_WRITE_TEXT);
    break;

  case LWS_CALLBACK_RECEIVE:  // the funny part
    {char* msg=(char*)in; msg[len]=0;}
    
    ProcShell::parseCommand((char*) in);
    break;

  case LWS_CALLBACK_CLOSED: { // the funny part
    printf("connection closed \n");
    ws_ctx._connections.erase(wsi);
    break;

  }
  default:
    break;
  }
    
  return 0;
}



void* WebSocketServer::_websocketThread(void* args){
  WebSocketServer* srv=(WebSocketServer*) args;
  std::cerr << "thred started" << endl;
  while (srv->_run) {
    ostringstream os;
    ProcShell::printStatusHTML(os);
    
    std::string buffer=os.str();
    srv->_client_response_length=buffer.length();
    memcpy(srv->_client_response_begin, buffer.c_str(), srv->_client_response_length);
    lws_callback_on_writable_all_protocol(srv->_context, &srv->_protocols[1]);
    lws_service(srv->_context, 10);
    usleep(100000);
  }
  lws_context_destroy(srv->_context);
  return 0;
}

void WebServer_start(int port, const char* resource_path){
  srv.start(port, resource_path);
}

void WebServer_stop(){
  srv.stop();
}
