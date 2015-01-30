
require 'pathname'

$:.unshift( (Pathname.new(__FILE__).realpath.dirname.parent+'utils').to_s )
$:.unshift Pathname.new(__FILE__).realpath.dirname.to_s

require 'ctty'

# this is like ./../../../
CAMBADA_PREFIX= Pathname.new(__FILE__).realpath.dirname.parent.parent.parent.parent.parent
CAMBADA_CONF= CAMBADA_PREFIX + 'config'
SIM_WORLDS= CAMBADA_CONF + 'worlds'

require 'fileutils.rb'

FileUtils.mkdir SIM_WORLDS.to_s unless SIM_WORLDS.exist?

def generate

  # First load the yaml file
  require 'yaml'
  yamlfile = CAMBADA_CONF+'sim.conf.yaml'
  unless yamlfile.exist?
    onoe "unable to find " + Tty.white + yamlfile.basename.to_s + Tty.reset + 
          "\nIt should be under " + Tty.em + yamlfile.dirname.to_s + Tty.reset
    return false
  end

  worldfile = CAMBADA_CONF+ 'worlds' + 'CAMBADA.world'
  must_update = false
  unless worldfile.exist?
    must_update = true
  else
    must_update = worldfile.mtime < yamlfile.mtime
  end

  if must_update
    say "Generating CAMBADA.world"
    yamlconf = YAML.load_file yamlfile.to_s
    require 'gen_world'
    Genworld.generate yamlconf
  end

end

## Call main function
generate
