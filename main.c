#include <u.h>
#include <libc.h>
#include <bio.h>
#include <thread.h>
#include <usb.h>

typedef struct Ups Ups;

struct Ups
{
	Dev *ctrl;
	Dev *intr;
	Biobuf *out;

	char *id;
	uint va;
	uint ncpus;
	uint nmaps;
	uint nalarms;
	uint cfgblksz;
	uint statmapsz;
	uint almlogsz;
	uint evtlogsz;
	uint topblksz;
	uint cmdlstsz;
	uint outblksz;
	uint almblksz;
};

static void
usage(void)
{
	fprint(2, "usage: %s dev\n", argv0);
	threadexitsall("usage");
}

static uchar
chksum(uchar *buf)
{
	int i;
	uchar c;

	c = 0;
	for(i = 0; i < buf[1]+2; i++)
		c -= buf[i];
	return c;
}


static int
setdesc(Dev *d, int val, int idx, uchar *buf, int len)
{
	return usbcmd(d, Rh2d|Rstd|Rdev, Rsetdesc, (val<<8)+idx, 0, buf, len);
}

static int
sendrdcmd(Dev *d, uchar cmd)
{
	uchar buf[4];

	buf[0] = 0xab;
	buf[1] = 1;
	buf[2] = cmd;
	buf[3] = chksum(buf);
	return setdesc(d, Dstr, 4, buf, 4);
}

static int
validpkt(uchar *pkt)
{
	int i;
	uchar c;

	c = 0;
	for(i = 0; i < 5+pkt[2]; i++)
		c += pkt[i];
	return c == 0;
}

static int
readresp(Biobuf *bio, uchar cmd, uchar *buf, int buflen)
{
	uchar sum, seq;
	int c, pktlen;
	uchar *bp, *bend;

	bp = buf;
	bend = buf + buflen;
Next:
	for(;;){
		if((c = Bgetc(bio)) < 0)
			return -1;
		if(c == 0xab)
			break;
	}
	sum = 0xab;
	if((c = Bgetc(bio)) < 0)
		return -1;
	if(c != cmd - 0x30)
		goto Next;
	sum += c;
	if((c = Bgetc(bio)) < 0)
		return -1;
	pktlen = c;
	sum += c;
	if((c = Bgetc(bio)) < 0)
		return -1;
	seq = c;
	sum += c;
	while(pktlen-- > 0){
		if(bp >= bend)
			return -1;
		if((c = Bgetc(bio)) < 0)
			return -1;
		*bp++ = c;
		sum += c;
	}
	if((c = Bgetc(bio)) < 0)
		return -1;
	sum += c;
	if(sum != 0)
		return -1;
	if((seq & 0x80) == 0)
		goto Next;
	return bp - buf;
}

int
pwread(Ups *ups, uchar cmd, uchar *buf, int len)
{
	if(sendrdcmd(ups->ctrl, cmd) < 0)
		return -1;
	return readresp(ups->out, cmd, buf, len);
}

static int
sendwrcmd(Dev *d, uchar *cmd, int cmdlen)
{
	uchar buf[128];

	buf[0] = 0xab;
	buf[1] = cmdlen;
	memmove(&buf[2], cmd, cmdlen);
	*(buf+2+cmdlen) = chksum(buf);
	return setdesc(d, Dstr, 4, buf, 2+cmdlen+1);
}

static char*
overallstatus(uchar v)
{
	switch(v){
	case 0xF0:
		return "ON BATTERY";
	case 0xE0:
		return "OUTPUT OVERLOAD";
	case 0xD0:
		return "RECTIFIER OVERLOAD";
	case 0x90:
		return "INVERTER RAMPING UP";
	case 0x80:
		return "SYNCING TO BYPASS";
	case 0x70:
		return "RECTIFIER RAMPING";
	case 0x64:
		return "ON MAINTENANCE BYPASS";
	case 0x63:
		return "ON BUCK/REDUCER";
	case 0x62:
		return "ON BOOST/STEP UP";
	case 0x61:
		return "ON DOUBLE BOOST";
	case 0x60:
		return "ON BYPASS";
	case 0x51:
		return "HIGH EFFICIENCY MODE";
	case 0x50:
		return "SYSTEM NORMAL";
	case 0x40:
		return "UPS SUPPORTING LOAD";
	case 0x30:
		return "UPS ON";
	case 0x21:
		return "OUTLET SWITCH OPEN";
	case 0x20:
		return "OUTLET BREAKER OPEN";
	case 0x11:
		return "MODULE FAILURE";
	case 0x10:
		return "UPS OFF";
	default:
		break;
	}
	return "UNKNOWN";
}

static char *topbits[8] = {
	"bypass-installed",
	"output-breaker-closed",
	"on-bypass",
	"on-battery",
	"inverter-on",
	"low-battery",
	"rectifier-on",
	"utility-present",
};

static char*
topologystr(uchar v)
{
	static char buf[1024];
	int i;
	char *s, *e;

	s = buf;
	e = buf + sizeof(buf);
	*s = 0;
	for(i = 0; i < 8; i++){
		if((v&(1<<i)) == 0)
			continue;
		s = seprint(s, e, ",%s", topbits[i]);
	}
	return buf+1;
}

static ushort
getu16(uchar *b)
{
	ushort s;

	s = b[1];
	s <<= 8;
	s |= b[0];
	return s;
}

static void
init(Ups *ups)
{
	int i, n;
	uint v;
	uchar buf[4096];

	n = pwread(ups, 0x31, buf, sizeof(buf));
	if(n < 0)
		sysfatal("pwread: %r");

	i = 0;
	ups->ncpus = buf[i];
	print("#cpus=%uhhd\n", buf[i]);

	i += 2*buf[i]+1;

	if(buf[i++] != 0){
		v = buf[i];
	}else{
		v = getu16(buf+i);
		i += 2;
	}

	ups->va = 50*v;
	print("%udVA\n", ups->va);

	//print("%uhhd output phase(s)°\n", buf[i]);
	i += 2;

	v = buf[i++];
	ups->id = mallocz(v+1, 1);
	strncpy(ups->id, (char*)buf+i, v);
	i += v;

	v = buf[i++];
	ups->nmaps = v;
	print("#maps=%uhhd\n", v);
	i += v;

	v = buf[i++];
	ups->nalarms = v;
	print("#alarms=%uhhd\n", v);
	i += v;

	v = getu16(buf+i);
	ups->cfgblksz = v;
	print("%uhd byte config block\n", v);
	i += 2;

	v = buf[i];
	ups->statmapsz = v;
	print("%uhhd byte statistics map\n", v);
	i += buf[i] + 1;

	v = getu16(buf+i);
	ups->almlogsz = v;
	print("%ud byte alarm log\n", v);
	i += 2;

	v = getu16(buf+i);
	ups->evtlogsz = v;
	print("%ud byte custom event log\n", v);
	i += 2;

	v = getu16(buf+i);
	ups->topblksz = v;
	print("%ud byte topology block\n", v);
	i += 2;

	i += 1;

	v = getu16(buf+i);
	ups->cmdlstsz = v;
	print("%ud byte command list block\n", v);
	i += 2;

	v = getu16(buf+i);
	ups->outblksz = v;
	print("%ud byte outlet monitoring block\n", v);
	i += 2;

	v = getu16(buf+i);
	ups->almblksz = v;
	print("%ud byte alarm block\n", v);
	i += 2;
	assert(i == n);
}

static void
stats(Ups *ups)
{
	int n;
	uchar buf[128];

	n = pwread(ups, 0x40, buf, sizeof(buf));
	if(n < 0)
		sysfatal("pwread: %r");

	/*
	fprint(2, "#cmds=%uhhd\n", buf[0]);
	for(i = 0; i < n; i++){
		fprint(2, "cmd[%d] = 0x%uhhx\n", i, buf[2+i]);
	}
	*/

	n = pwread(ups, 0x33, buf, sizeof(buf));
	if(n < 0)
		sysfatal("pwread: %r");
	print("Overall Status: %s\n", overallstatus(buf[0]));
	print("Topology Status: %s\n", topologystr(buf[1]));

	n = pwread(ups, 0x34, buf, sizeof(buf));
	if(n < 0)
		sysfatal("pwread: %r");
}

void
threadmain(int argc, char **argv)
{
	int i;
	Ups ups;
	Ep *ep;
	Usbdev *ud;

	ARGBEGIN{
	default:
		usage();
	}ARGEND;

	if(argc != 1)
		usage();
	ups.ctrl = getdev(*argv);
	if(ups.ctrl == nil)
		sysfatal("getdev: %r");
	ud = ups.ctrl->usb;
	ups.intr = nil;
	for(i = 0; i < nelem(ud->ep); i++){
		if((ep = ud->ep[i]) == nil)
			continue;
		if(ep->type == Eintr && ep->dir == Ein)
			ups.intr = openep(ups.ctrl, ep->id);
	}
	if(ups.intr == nil)
		sysfatal("could not find interrupt endpoint");
	if(opendevdata(ups.intr, OREAD) < 0)
		sysfatal("could not open interrupt endpoint: %r");
	ups.out = Bfdopen(ups.intr->dfd, OREAD);
	if(ups.out == nil)
		sysfatal("open: %r");
	//init(&ups);
	stats(&ups);
}
