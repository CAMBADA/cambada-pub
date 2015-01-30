
# I got this from "Homebrew"
# You should check it out => http://github.com/mxcl/homebrew
#

class Tty

  class << self
    def blue; bold 34; end
    def white; bold 39; end
    def red; underline 31; end
    def yellow; underline 33; end
    def reset; escape 0; end
    def em; underline 39; end

    private
    def color c
      escape "0;#{c}"
    end

    def bold b
      escape "1;#{b}"
    end

    def underline u
      escape "4;#{u}"
    end

    def escape e
      "\033[#{e}m" if $stdout.tty?
    end
  end # End of class functions

end

def say title, *things
  puts "#{Tty.blue}==>#{Tty.white} #{title}#{Tty.reset}"
  puts *things unless things.empty?
end

def opoo warning
    puts "#{Tty.red}Warning#{Tty.reset}: #{warning}"
end

def onoe error
    lines = error.to_s.split'\n'
      puts "#{Tty.red}Error#{Tty.reset}: #{lines.shift}"
        puts *lines unless lines.empty?
end

