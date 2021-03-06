# FRC-2021

Code for the 2021 FIRST Robotics Competition based on the 2020 code (pulled from CompetitionBotBranch).

## Git explained

The "Pro Git" book is an excellent piece of free documentation for Git.  I highly recommend you read it, because if you do, you will have a comprehensive knowledge of git.

The Pro Git book can be found [here.](https://git-scm.com/book/en/v2)
Additionally, please check out this article on writing commits: [How to Write a Git Commit Message](https://chris.beams.io/posts/git-commit/).  Although having an expansive body as described in that post is less vital for us, *please* follow the rules for the subject line.  That means never titling them "Commit": **ever.**

Lastly, we need some degree of configuration to be done by you.  This will require you to use the command line a little.
Don't worry! Its all **totally** safe.  By doing this, we can get a cleaner git history, which makes it a lot easier to find problems.

## How To Get Started With Our Code

This handy guide will cover ally ou need

#### Get the dependencies

Firstly, ensure that you have setup a full FRC programming environemnt.  That includes an installation of FRC VsCode and WPILib.  Next, install a copy of Git.  You can find one at [git's homepage](https://git-scm.com/downloads).  You will also need to get a copy of the CTRE dependencies, but that's only really important if your working with the robot, and all team computers should have it already.

#### Get the Code

After installing all the needed dependencies, open up VS Code. This is where the non-standard stuff begins.  Clone this repository by hitting Ctrl-Shift-P to open the command palette, then type in 'clone'.  You should get a result like "git clone".  Click it, and enter the repostory url, `https://github.com/FRC999/FRC-2021.git`.  Then press enter, select a location for the code, and confirm that you want to start working on it

#### Setup our custom configuration

At the bottom of your VSCode screen, there should be a few tabs, one of which says 'terminal'.  If there isn't, drag up from that bottom bar, and select the terminal view.  You should now have you code screen split in half, with the terminal below and the code above.  Congrats!  You have opened the command line.

While we're here, we need to set up two things.  The first is the typical git configuration, entering your name and email, and might already be done.  The second is new for 2021, and is definitly not already done.  **Veterans and beginners alike should do this second change**.

##### Name and Email config

This can also be done through the GUI, by searching 'git gui' in the Start menu, and selecting 'edit preferences'.  However, doing it through the command line is more powerful, since you learn more about how Git works.

To setup your name and email, run
`git config --global user.name "FIRSTNAME LASTNAME"`
and
`git config --global user.email "youremail@gmail.com"`
For instance, I, Calum McConnell, would run
`git config --global user.name "Calum McConnell"` and `git config --global user.email "calumlikesapplepie@gmail.com"`

This command has a few parts.  First, `git` specifies that we want the computer to work with Git, as opposed to any of the other programs you have.  `config` indicates that we are CONFIGuring Git to behave a certain way.  `--global` means that we want the configuration to be true across this computer, not just for this repository.  `user.` means that we wish to set variables in the User section of git's config, and `name` means the variable (or 'key') that we wish to set is the name variable.  The reason the name itself is in quotes is because otherwise Git might think that we are specifying two names, Calum and McConnell, and hold both in its datastore.

##### Git Pull configuration

In previous years, we have consistently used Git's default behavior when updating the local copy (ie, the verision on your machine).  When you update, or 'pull', it is sometimes the case that the verison stored on Github's server (or 'remote') has changed since you started editing.  In that case, Git has to run a merge, to recombine the two versions.  By default, git does this by creating a new commit (more on that later), which combines the two previous commits.  That means the commit history rapidly becomes convoluted, with branches forming and recombining at random.  This makes it hard to look back, and see what happened.

However, git provides an alternative behavior, which is to 'rebase' your commits (ie, replay them on top of the other new changes).  This produces a more readable history, and lacks any downside for us.  To set this up, run

`git config --local pull.rebase true`

This command's first part, `git config` means to run Git's configuration system and make changes to how Git is set up.  `--local` means to only apply changes to this repository: you may wish to use `--global` instead, but you don't need to.  `pull.rebase` means to change the rebase variable of the pull section, and `true` means to make that variable, well, true.

I **highly reccomend** that **_everyone_** run the above command.  It will make our git history significantly easier to understand, which will aid any debugging we need to do.  I will know if you don't run that command: I will know and I will be sad.



## How Git Works for the busy Gits who won't read the book

Git has four main types of objects that I am going to talk about: trees, commits, branches, and blobs.  A blob is a file: git keeps track of them (and most other objects) by making a hash of them, which is a way to assign a unique identifier to a file based solely on its content.  Remember that word, hash: git hashes a lot of things.

So, a tree is the directory structure: think of it as a tree, whose base is the root direction (get it, root?), and the branches are subdirectories, with the leaves being blobs (files).  Now, remember how I said that git tracks objects by their hashes?  Well, what that means is that if a file is unchanged, git will not store another copy: rather, git will just reference the previous copy.

(It's worth noting that just doing that is inefficient: after all, changing one character in a large file will drastically change the hash, and so the large file would be stored all over again.  Git has a quite clever way of packing those together: it's called packing, and it's what is going on whenever git talks about deltas)

Now, the next up in the hierarchy of git objects is the commit.  A commit contains a pointer to a tree, but it also states what commit it is based on, who committed it and when, and a custom message that they would have added.  Commits are what you work with a lot in git: while trees and blobs are good to know about, because they are how git works, it's important to understand commits first.  However, the author is getting tired, and so they won't go further.

*TODO: expand on the above, and explain branches and such*

### Git Branch Structure

This git repository is designed to allow us to track the various states of confidence
that we have in code with more fidelity than simply "completed" and "in progress".
Last year, we discovered that the code in our master branch was nonfunctional: we thus
did all of our development in a second branch, HartfordCode, leaving master as a
useless relic.

Thus, I am introducing a new system of branching, with three main branches, and all
development taking place in sub-branches.  We attempted to do development in sub-branches
last year, however, this year, we will do this slightly differently.  The main branches are
as follows: (totally not based on the three-releases model of Debian)

#### 'Stable' branch

The *Stable* branch is the most stable of the branches: it is only for code that has been
both thoroughly tested and documented, and can be considered "competition ready".  **It
must not be directly committed to**, except in cases of vital bugfixes: instead, bring
in changes from 'master'.  *Stable* is **intentionally** distinct from master; Code
in *master* should be tested and stable, but code in *stable* should be extremely tested
and very stable.  Additionally, release tags should be made on stable branch heads.

#### 'Master' branch

*Master* branch is just a typical master branch (it happens to be the equivalent of Debian
testing).  Thus, it is code that can be loaded on to the robot before a competition and
expected to run.  Commits directly to *master* are permitted, but only if they are tested
(eg, bugfixes made during a testing session), and are not recommended (you should make the
fixes before code enters master).  The *master* branch can only be set to commits that are
known to work: period.  If master is being updated to a less-well-tested version, it is
recommended to update stable to point to where master was.  If the code currently in stable
is still desired, a release tag should be used.

#### 'Sid' branch

*Sid* is based on the debian branch of the same name.  It is where code suitable  for being
tested is put.  Code in *sid* may be buggy: it should not be used for a competition.
*Sid* may be directly committed to as part of a bugfixes, but commits to *sid* should
try to leave *sid* in a state that is somewhat suitable for a competition bot.  Thus,
new features should be developed in separate branches, and then transferred over to *sid*.

### Git submodules

We use(d) a git submodule to bring in Witchcraft, our reusable set of java classes.  I'll
write more on that later: currently, I need to wrap up kickoff day preparations.
