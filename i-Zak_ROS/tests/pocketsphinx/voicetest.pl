#!/usr/bin/perl


system("rm -f recout.txt 2>/dev/null");
system("touch recout.txt");


system("rostopic echo /recognizer/output >> recout.txt &");

open TIN, "< testplan.txt";
open IN,"< recout.txt";

open OUT,"> results.txt";

my $tot=0;

while (<TIN>) {
    next if ($_ eq "");

    my @words=split(/ /,$_);
    my $wavfile=@words[0];
    shift(@words);
    my $itext=join(" ",@words);

    sleep 1;
    system("playback $wavfile");
    sleep 1;

    while (<IN>) {
	if ($_  =~ /data:.*/) {
	    $text=(split(/data: /,$_))[1];
	    print "I got:".$text;
	}
    }

    print OUT "$wavfile\n$itext$text";

    if ($itext eq $text) {
	print OUT "+1\n\n";
	$tot++;
    }
    else {
	print OUT "0\n\n";
    }

}

print OUT "Final score: $tot\n";

close TIN;

system("killall -9 rostopic");

close IN;

system("rm -f recout.txt 2>/dev/null");





