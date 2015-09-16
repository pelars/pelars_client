#pragma once
#include "mongoose.h"
#include <iostream>

// MONGOOSE SERVER
/*
bool mg_thread_stop = false;

// Poll the server to check if there are pending requests
void * serve(void * server) 
{
  for (;!mg_thread_stop;) mg_poll_server((struct mg_server *) server, 1000);
  return NULL;
}

// Mongoose event handler (an http request for example)
int ev_handler(struct mg_connection * conn, enum mg_event ev) 
{
  //TODO meaningful debug has to be done here
  if (ev == MG_REQUEST) {
    int * session = (int *)conn->server_param;
    //mg_send_header(conn, "Content-Type", "text/plain");
    mg_printf_data(conn, "%d", *session);
    //mg_send_data(conn, (void *)session, sizeof(int));
    return MG_TRUE;
  } else if (ev == MG_AUTH) {
    return MG_TRUE;
  } else {
    return MG_FALSE;
  }
}
*/
void ev_handler(struct mg_connection *nc, int ev, void *p) {
  int * session = (int *)nc->mgr->user_data;
  std::cout << ev << std::endl;
  switch (ev) {
    case 4:
      mg_printf(nc, "%d", *session);
      break;
  }
}
