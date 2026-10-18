/*
 *	comm_json.c
 *	Subsystem 1 -- a tiny JSON writer that can never overrun its buffer.
 *
 *	Why not tm_sprintf()?  It has no length limit (one long string and it
 *	writes past the end of the buffer) and no %f.  This writer checks
 *	every byte, and if anything does not fit it marks the message as
 *	overflowed so json_end() returns -1 and the caller drops it, instead
 *	of publishing half a JSON object.
 *
 *	Usage:
 *	    JSON_BUF jb;
 *	    json_begin(&jb, buf, sizeof(buf));     ->  {
 *	    json_int(&jb, "seq", 12);              ->  {"seq":12
 *	    json_obj_open(&jb, "line");            ->  {"seq":12,"line":{
 *	    json_int(&jb, "l", 812);               ->  ...{"l":812
 *	    json_obj_close(&jb);                   ->  ...{"l":812}
 *	    len = json_end(&jb);                   ->  {"seq":12,"line":{"l":812}}
 */

#include "s1_private.h"

#define INT_TEXT_MAX	12	/* "-2147483648" + spare */

/* Append one byte, keeping room for the final NUL */
LOCAL void put_ch(JSON_BUF *jb, UB c)
{
	if (jb->len < (jb->cap - 1)) {
		jb->buf[jb->len] = c;
		jb->len++;
		jb->buf[jb->len] = '\0';
	} else {
		jb->overflow = TRUE;
	}
}

LOCAL void put_raw(JSON_BUF *jb, const char *s)
{
	while (*s != '\0') {
		put_ch(jb, (UB)*s);
		s++;
	}
}

/* A string value with the characters JSON requires escaped */
LOCAL void put_quoted(JSON_BUF *jb, const char *s)
{
	put_ch(jb, '"');
	while (*s != '\0') {
		UB c = (UB)*s;

		if ((c == '"') || (c == '\\')) {
			put_ch(jb, '\\');
			put_ch(jb, c);
		} else if ((c < 0x20U) || (c >= 0x7fU)) {
			put_ch(jb, '?');	/* keep the payload plain ASCII */
		} else {
			put_ch(jb, c);
		}
		s++;
	}
	put_ch(jb, '"');
}

LOCAL void put_uint(JSON_BUF *jb, UW v)
{
	char	tmp[INT_TEXT_MAX];
	INT	n = 0;

	/* Digits come out backwards: 123 -> '3','2','1' */
	do {
		tmp[n] = (char)('0' + (v % 10U));
		n++;
		v /= 10U;
	} while ((v != 0U) && (n < INT_TEXT_MAX));

	while (n > 0) {
		n--;
		put_ch(jb, (UB)tmp[n]);
	}
}

LOCAL void put_int(JSON_BUF *jb, W v)
{
	if (v < 0) {
		put_ch(jb, '-');
		/* -(v+1)+1 avoids overflow for the most negative value */
		put_uint(jb, (UW)(-(v + 1)) + 1U);
	} else {
		put_uint(jb, (UW)v);
	}
}

/* Comma (if not the first member) then "key": */
LOCAL void put_key(JSON_BUF *jb, const char *key)
{
	if (jb->len > 0) {
		UB last = jb->buf[jb->len - 1];
		if ((last != '{') && (last != '[')) {
			put_ch(jb, ',');
		}
	}
	put_quoted(jb, key);
	put_ch(jb, ':');
}

EXPORT void json_begin(JSON_BUF *jb, UB *buf, INT cap)
{
	jb->buf      = buf;
	jb->cap      = cap;
	jb->len      = 0;
	jb->overflow = (cap < 3) ? TRUE : FALSE;	/* "{}" + NUL */
	if (cap > 0) {
		buf[0] = '\0';
	}
	put_ch(jb, '{');
}

EXPORT void json_int(JSON_BUF *jb, const char *key, W value)
{
	put_key(jb, key);
	put_int(jb, value);
}

EXPORT void json_uint(JSON_BUF *jb, const char *key, UW value)
{
	put_key(jb, key);
	put_uint(jb, value);
}

EXPORT void json_str(JSON_BUF *jb, const char *key, const char *value)
{
	put_key(jb, key);
	put_quoted(jb, (value != NULL) ? value : "");
}

EXPORT void json_bool(JSON_BUF *jb, const char *key, BOOL value)
{
	put_key(jb, key);
	put_raw(jb, value ? "true" : "false");
}

EXPORT void json_pair(JSON_BUF *jb, const char *key, W a, W b)
{
	put_key(jb, key);
	put_ch(jb, '[');
	put_int(jb, a);
	put_ch(jb, ',');
	put_int(jb, b);
	put_ch(jb, ']');
}

EXPORT void json_obj_open(JSON_BUF *jb, const char *key)
{
	put_key(jb, key);
	put_ch(jb, '{');
}

EXPORT void json_obj_close(JSON_BUF *jb)
{
	put_ch(jb, '}');
}

EXPORT INT json_end(JSON_BUF *jb)
{
	put_ch(jb, '}');
	return jb->overflow ? -1 : jb->len;
}
