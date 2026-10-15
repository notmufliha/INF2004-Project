/*
 *	comm_cmd.c
 *	Subsystem 1 -- remote command parser and validator (FR-B1-5,
 *	NFR-B1-5).
 *
 *	Command format (plain text, case does not matter):
 *
 *	    VERB [ARG] [#SEQ]
 *
 *	    START            STOP             ESTOP
 *	    PAUSE            RESUME           RESET_PEAK
 *	    MODE AUTO | LINE | TEST
 *	    SPEED 0..100                     (percent)
 *	    PING             STATUS           TELEM 100..10000   (handled by S1)
 *
 *	    "#SEQ" is optional, e.g.  "STOP #17".  It is echoed back in the
 *	    acknowledgement so the sender can match replies to requests.
 *
 *	Anything else is REJECTED and never reaches the Vehicle Controller.
 *	This file has no kernel calls and no globals, so it can be unit-tested
 *	directly (subsystem1_test.c, S1-U03/S1-U04).
 */

#include "s1_private.h"

#define WORD_MAX	12	/* longest verb/argument we accept */
#define SPEED_MAX	100
#define SEQ_MAX		99999999

typedef struct {
	const char	*verb;
	UB		type;		/* REMOTE_CMD_TYPE, or RCMD_NONE */
	UB		local;		/* LOCAL_CMD                     */
	UB		arg_kind;	/* ARG_xxx                       */
} CMD_DEF;

#define ARG_NONE	0
#define ARG_SPEED	1
#define ARG_MODE	2
#define ARG_TELEM	3

LOCAL const CMD_DEF cmd_table[] = {
	{ "START",	RCMD_START,	 LOCAL_NONE,   ARG_NONE  },
	{ "STOP",	RCMD_STOP,	 LOCAL_NONE,   ARG_NONE  },
	{ "ESTOP",	RCMD_ESTOP,	 LOCAL_NONE,   ARG_NONE  },
	{ "PAUSE",	RCMD_PAUSE,	 LOCAL_NONE,   ARG_NONE  },
	{ "RESUME",	RCMD_RESUME,	 LOCAL_NONE,   ARG_NONE  },
	{ "RESET_PEAK",	RCMD_RESET_PEAK, LOCAL_NONE,   ARG_NONE  },
	{ "MODE",	RCMD_MODE,	 LOCAL_NONE,   ARG_MODE  },
	{ "SPEED",	RCMD_SPEED,	 LOCAL_NONE,   ARG_SPEED },
	{ "PING",	RCMD_NONE,	 LOCAL_PING,   ARG_NONE  },
	{ "STATUS",	RCMD_NONE,	 LOCAL_STATUS, ARG_NONE  },
	{ "TELEM",	RCMD_NONE,	 LOCAL_TELEM,  ARG_TELEM },
};
#define CMD_TABLE_LEN	((INT)(sizeof(cmd_table) / sizeof(cmd_table[0])))

LOCAL const char *const mode_words[MODE_COUNT] = { "AUTO", "LINE", "TEST" };

LOCAL const char *const result_text[CMD_ERR_COUNT] = {
	"ok",
	"empty command",
	"command too long",
	"non-printable character",
	"unknown command",
	"missing argument",
	"unexpected extra argument",
	"argument is not a number",
	"argument out of range",
	"unknown mode (AUTO, LINE, TEST)",
};

EXPORT const char *cmd_result_text(CMD_RESULT r)
{
	return ((UINT)r < (UINT)CMD_ERR_COUNT) ? result_text[r] : "?";
}

LOCAL UB to_upper(UB c)
{
	return ((c >= 'a') && (c <= 'z')) ? (UB)(c - ('a' - 'A')) : c;
}

LOCAL BOOL is_space(UB c)
{
	return ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n'))
		? TRUE : FALSE;
}

LOCAL BOOL str_eq(const UB *a, const char *b)
{
	while ((*a != '\0') && (*b != '\0')) {
		if (*a != (UB)*b) {
			return FALSE;
		}
		a++;
		b++;
	}
	return ((*a == '\0') && (*b == '\0')) ? TRUE : FALSE;
}

/* Decimal digits only, no sign.  FALSE if not a number or too big. */
LOCAL BOOL parse_uint(const UB *s, W max, W *out)
{
	W v = 0;

	if (*s == '\0') {
		return FALSE;
	}
	while (*s != '\0') {
		if ((*s < '0') || (*s > '9')) {
			return FALSE;
		}
		v = (v * 10) + (W)(*s - '0');
		if (v > max) {
			return FALSE;
		}
		s++;
	}
	*out = v;
	return TRUE;
}

EXPORT CMD_RESULT comm_cmd_parse(const UB *text, INT len, PARSED_CMD *out)
{
	UB		words[3][WORD_MAX + 1];
	INT		nwords = 0;
	INT		wlen = 0;
	INT		i;
	W		seq = 0;
	W		num = 0;
	const CMD_DEF	*def = NULL;

	out->cmd.type     = RCMD_NONE;
	out->cmd.reserved = 0;
	out->cmd.arg      = 0;
	out->cmd.seq      = 0;
	out->local        = LOCAL_NONE;
	out->local_arg    = 0;

	if (len > COMM_CMD_MAX) {
		return CMD_ERR_TOO_LONG;
	}

	/* 1. Split into upper-case words.  At most 3: VERB ARG #SEQ */
	for (i = 0; i <= len; i++) {
		UB c = (i < len) ? text[i] : (UB)' ';	/* flush last word */

		if ((i < len) && (c == '\0')) {
			c = ' ';
		}
		if (is_space(c)) {
			if (wlen > 0) {
				words[nwords][wlen] = '\0';
				nwords++;
				wlen = 0;
			}
			continue;
		}
		if ((c < 0x21U) || (c > 0x7eU)) {
			return CMD_ERR_BAD_CHAR;
		}
		if (nwords >= 3) {
			return CMD_ERR_EXTRA_ARG;
		}
		if (wlen >= WORD_MAX) {
			return CMD_ERR_TOO_LONG;
		}
		words[nwords][wlen] = to_upper(c);
		wlen++;
	}
	if (nwords == 0) {
		return CMD_ERR_EMPTY;
	}

	/* 2. Optional trailing "#SEQ" */
	if (words[nwords - 1][0] == '#') {
		if (!parse_uint(&words[nwords - 1][1], SEQ_MAX, &seq)) {
			return CMD_ERR_BAD_NUMBER;
		}
		nwords--;
		if (nwords == 0) {
			return CMD_ERR_EMPTY;
		}
	}

	/* 3. Look up the verb */
	for (i = 0; i < CMD_TABLE_LEN; i++) {
		if (str_eq(words[0], cmd_table[i].verb)) {
			def = &cmd_table[i];
			break;
		}
	}
	if (def == NULL) {
		return CMD_ERR_UNKNOWN;
	}

	/* 4. Check the argument count and value */
	if ((def->arg_kind == ARG_NONE) && (nwords != 1)) {
		return CMD_ERR_EXTRA_ARG;
	}
	if ((def->arg_kind != ARG_NONE) && (nwords < 2)) {
		return CMD_ERR_MISSING_ARG;
	}
	if (nwords > 2) {
		return CMD_ERR_EXTRA_ARG;
	}

	switch (def->arg_kind) {
	case ARG_SPEED:
		if (!parse_uint(words[1], SEQ_MAX, &num)) {
			return CMD_ERR_BAD_NUMBER;
		}
		if (num > SPEED_MAX) {
			return CMD_ERR_RANGE;
		}
		break;

	case ARG_TELEM:
		if (!parse_uint(words[1], SEQ_MAX, &num)) {
			return CMD_ERR_BAD_NUMBER;
		}
		if ((num < COMM_TELEM_MIN_MS) || (num > COMM_TELEM_MAX_MS)) {
			return CMD_ERR_RANGE;
		}
		break;

	case ARG_MODE:
		num = -1;
		for (i = 0; i < MODE_COUNT; i++) {
			if (str_eq(words[1], mode_words[i])) {
				num = i;
				break;
			}
		}
		if (num < 0) {
			return CMD_ERR_BAD_MODE;
		}
		break;

	default:
		break;
	}

	/* 5. Valid: fill in the result */
	out->local = (LOCAL_CMD)def->local;
	if (def->local == LOCAL_NONE) {
		out->cmd.type = def->type;
		out->cmd.arg  = (H)num;
		out->cmd.seq  = (UW)seq;
	} else {
		out->cmd.seq   = (UW)seq;	/* still echoed in the ack */
		out->local_arg = num;
	}
	return CMD_OK;
}
