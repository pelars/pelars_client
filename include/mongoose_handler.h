#pragma once
#include "mongoose.h"

// MONGOOSE SERVER

bool mg_thread_stop = false;

// Poll the server to check if there are pending requests
static void * serve(void * server) 
{
  for (;!mg_thread_stop;) mg_poll_server((struct mg_server *) server, 1000);
  return NULL;
}

// Mongoose event handler (an http request for example)
static int ev_handler(struct mg_connection * conn, enum mg_event ev) 
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
