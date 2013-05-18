#! /perl
# type Documents\Robotics\isobot_ir_codes | \bin\isob2.pl
use strict;
my $showMatrix=1;
my $outputEquations=1;
my %f;
my @cmds;

sub gray($)
{
   my $n = shift;
   my $g = 0;
   $g += $n & 8;
   $g += ($n & 4) ^ (($n>>1) & 4);
   $g += ($n & 2) ^ (($n>>1) & 2);
   $g += ($n & 1) ^ (($n>>1) & 1);
   return $g;
}


sub bin2num($)
{
	my $v = 0;
	my $s = shift;
	for(my $i=0; $i < length($s); $i++ )
	{
		$v *= 2;
		$v ++ if substr($s,$i,1) == '1';
	}
	return $v;
}

sub num2bin($$)
{
	my ($n,$len) = @_;
	my $s = "";
	my $mask = 1;
	while($len>0)
	{
		$s = (($n & $mask)?'1':'0') . $s;
		$len--; 
		$mask <<=1;
	}
	return $s;
}

sub num2str($$$)
{
	my ($n,$base,$len) = @_;
	my $s = "";
	while($n>0)	
	{
		my $m = $n % $base;
		$n = int($n/$base);
		my $d = ($m < 10) ? chr(ord('0')+$m) : chr(ord('A') + $m - 10);
		$s = $d . $s;
	}
	while(length($s)<$len) { $s = '0' . $s; }
	return $s;
}

sub revstr($)
{
	my $s = shift;
	my $sr = '';
	for(my $i = length($s)-1; $i >= 0; $i --)
	{
		$sr .= substr($s,$i,1);
	}
	return $sr;
}

sub csumby3bits($)
{
	my $v = shift;
	my $s = 0;
	while ($v>0)
	{
		$s += $v & 7;
		$v >>= 3;
	}
	return $s;
}

# produce checksum for the message
# assumes theat the checksum in the header byte is 0
sub csum($)
{
	my $cmd = shift;
	my $s = 0;
	# first sum all the bytes of the message
	my $val = $cmd + ($cmd >> 8) + ($cmd >> 16) + ($cmd >> 24);
	$val &= 0xFF;
	# then sum the result 3 bits at a time to produce 3 bit checksum
	while($val > 0)
	{
		$s += ($val & 0x7);
		$val >>= 3;
	}
	return $s;
}

# formats cmd for printout
sub cmd2str($)
{
	my $cmd = shift;
	my $str = num2str($cmd,2,30);
	return substr($str,0,1).'.'.substr($str,1,2).'.'.substr($str,3,3).'.'.
	       substr($str,6,8).'.'.substr($str,14,8).'.'.substr($str,22,8);
}

# adds label (name) and cmd to the database 
sub addData($$)
{
	my ($name,$cmdstr) = @_;
	return if !defined($cmdstr) or $cmdstr eq "";
	my %x;
	$x{NAME} = $name;
	$x{PATT} = $cmdstr;
	# remove separating dots if any
	$cmdstr =~ s/\.//gm;
	return if length($cmdstr) < 22;
	my $code = bin2num($cmdstr);
	$code <<= 8 if( length($cmdstr) <= 22 ); # 2 bytes va 3 bytes message
#	print num2str($code,2,30)."\n"; #if length($cmdstr)>22;
	$x{CODE} = $code;
	my $hdr = $code >> 24;
	$x{HEAD} = $hdr;
	$x{TYPE} = ($hdr >> 3) & 0x3;
	$x{CHAN} = $hdr >> 5;
	$x{CSUM} = $hdr & 0x7;
	$x{CMD1} = ($code >> 16) & 0xFF;
	$x{CMD2} = ($code >>  8) & 0xFF;
	$x{CMD3} = $code & 0xFF;
	push @cmds,\%x; # if $x{CHAN}==0;
	$f{int $x{CMD1}}=\%x;
#	print "$x{CMD1} $x{CODE}\n";
}

############################################################################

### read data from stdin
while(<>)
{
	# 0.01.101.11010011.00001011
	if( /(\S+)\s*([01.]+)/ )
	{
		addData($1,$2);
	}
	elsif( /(\S+)\s*(\d\.\d\d\.\d{3}\.\d{8}\.\d{8})\s/ )
	{	
		addData($1,$2);
	}
	elsif( /(\S+)\s*(\d\d\d\d{3}\d{8}\d{8})/ )
	{
		addData($1,$2);
	}
}

### output C header
if(0)
{
	foreach my $cmd (@cmds)
	{
		my %c = %{$cmd};
		print sprintf("CMD_%s\t0x%06x // %02x %s %s\n",
		$c{NAME}, $c{CODE}, ($c{CODE}>>8)&0xFF, num2str($c{CODE},2,22));
	}
}
if(1)
{
	foreach my $cmd (sort {$a <=> $b} keys %f)
	{
		my %c = %{$f{$cmd}};
		print sprintf("#define CMD_%s\t0x%02x\n",$c{NAME}, $c{CMD1}) if( $c{TYPE} == 1 );
	}
	foreach my $cmd (@cmds)
	{
		my %c = %{$cmd};
		print sprintf("#define MSG_%s\t0x%02x%02x%02x\n",$c{NAME}, $c{CMD1}, $c{CMD2}, $c{CMD3}) if( $c{TYPE} == 0 );
	}
}

#output missing commands
my $prev = -1;
foreach my $cmd (sort {$a <=> $b} keys %f)
{
	if( $cmd-$prev > 1 )
	{
	    my ($g1,$g2)=($prev+1,$cmd-1);
		print "$g1-$g2 " ;
		print sprintf("%02X-%02X\n",$g1,$g2);
	}
	$prev = $cmd;
   # my %c = $f{$cmd};
	#print "$c{CMD}\n";	
}

if(0)
{
# output checksum check
foreach my $cmd (@cmds) # (sort { $a <=> $b} keys %f)
{
	my %c = %{$cmd}; #%{$f{$cmd}};
	print sprintf("%02X",$c{CMD});
	#print " $c{CHAN} $c{TYPE} ";
	print " $c{CSUM} ";
	my $cs = csum($c{CODE} & ~(7<<24));
	my $cs3 = $cs & 7;			
	my $diff = $c{CSUM}-($cs3);
	$diff += 8 if $diff < 0;
	$diff = 8 - $diff if $diff > 3;
	my $ok = ($diff==0 ? '  ' : sprintf("%2d",$diff));
	print " $cs3 $ok ";
	#my $cs3r = csumby3bits($c{REVCODE} & ~(7<<3)) & 7;
	#my $rc = num2str($c{REVCODE}& ~(7<<3),2,22);
	#print " $cs3r $rc ";
	#print num2str($cs,2,8);
	#print ' ';
	#print num2str($c{CODE},2,22);
	print cmd2str($c{CODE});
	print "\n";
}
}