#pragma once
//#include <stdlib.h>
#include <argp.h>

const char *argp_program_version =
  "v1";
const char *argp_program_bug_address =
  "giacomo.dabisias@gmail.com";

/* Program documentation. */
static char doc[] =
  "Pelars sensor";

/* A description of the arguments we accept. */
static char args_doc[] = "Path to template list file";

/* The options we understand. */
static struct argp_option options[] = {
  {"face",  'f', 0,      0,  "Record faces" },
  {"hand",    'h', 0,      0,  "Record markers for hand detection" },
  {"object",    'o', "template list file",      0,  "Record objects from a template list file" },
  {"visualization",  'v', 0,      0,  "Visualize windows" },
  {"particle",  'p',  0,  0,  "Starts particle recorder"},
  {"keylog",  'k', 0, 0,  "Sends activity data"},
  {"special",  's', 0, 0,  "handles signals"},
  {"ide",  'i', 0, 0,  "handles ide messages"},
  { 0 }
};

/* Used by main to communicate with parse_opt. */
struct arguments
{
  char *args[1];                /* arg1 & arg2 */
  bool face, hand, object, visualization, particle, keylog, special, ide;
  std::string template_file;
};

/* Parse a single option. */
static error_t
parse_opt (int key, char *arg, struct argp_state *state)
{
  /* Get the input argument from argp_parse, which we
     know is a pointer to our arguments structure. */
  struct arguments *arguments = (struct arguments*)state->input;

  switch (key)
    {
    case 'h': 
      arguments->hand = true;
      break;
    case 'o':
      arguments->object = true;
      arguments->template_file = arg;
      break;
    case 'f':
      arguments->face = true;
      break;
    case 'v':
      arguments->visualization = true;
      break;
    case 'p':
      arguments->particle = true;
      break;
    case 'k':
      arguments->keylog = true;
      break;
    case 's':
      arguments->special = true;
      break;
    case 'i':
      arguments->ide = true;
      break;

    case ARGP_KEY_ARG:
        /* Too many arguments. */
        argp_usage (state);
      //arguments->args[state->arg_num] = arg;
      break;

    default:
      return ARGP_ERR_UNKNOWN;
    }
  return 0;
}

/* Our argp parser. */
static struct argp argp = { options, parse_opt, args_doc, doc };