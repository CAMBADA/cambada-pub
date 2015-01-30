require 'pathname'
require 'erb'

module Genworld

  # this is like ./../../../../../
  CAMBADA_PREFIX= Pathname.new(__FILE__).realpath.dirname.parent.parent.parent.parent.parent
  CAMBADA_CONF= CAMBADA_PREFIX + 'config'
  SIM_WORLDS= CAMBADA_CONF + 'worlds'

  @@conf = nil

  def self.generate yamlconf
    @@conf = yamlconf
    
    worldfile = SIM_WORLDS + 'CAMBADA.world'
    self.load_field if @@conf['field']['getfromCAMBADA']
    template = ERB.new XMLDATA
    simconf = @@conf
    File.open( worldfile.to_s, 'w' ).puts template.result(binding)
    true
  end
  
  private
  
  def self.load_field
    require 'rexml/document'
    toCollect = [ "ball_diameter",
                  "center_circle_radius",
                  "corner_arc_radius",
                  "field_length",
                  "field_width",
                  "goal_area_length",
                  "goal_area_width",
                  "penalty_area_length",
                  "penalty_area_width",
                  "theNorth"]


    cambada = CAMBADA_CONF + 'cambada.conf.xml'
    xmlFile = File.new( cambada )
    xmlDoc  = REXML::Document.new xmlFile

    fieldXML    = xmlDoc.elements.each("//Field"){|elem|}.select { |elem| toCollect.include? elem.attributes["name"]  }
    fieldValues = {}
    fieldXML.each {|elem| fieldValues[ elem.attributes["name"] ] = elem.attributes["value"]  }
    
    @@conf['field']['length'] = fieldValues["field_length"].to_i / 1000.0
    @@conf['field']['width']  = fieldValues["field_width"].to_i  / 1000.0
    @@conf['field']['center_radius'] = fieldValues["center_circle_radius"].to_i / 1000.0
    @@conf['field']['goalie_length'] = fieldValues["goal_area_length"].to_i     / 1000.0
    @@conf['field']['goalie_width']  = fieldValues["goal_area_width"].to_i      / 1000.0
    @@conf['field']['penalty_length']= fieldValues["penalty_area_length"].to_i  / 1000.0
    @@conf['field']['penalty_width'] = fieldValues["penalty_area_width"].to_i   / 1000.0
    @@conf['field']['theNorth']      = fieldValues["theNorth"].to_i

  end

end

XMLDATA = <<ENDDATA
<?xml version="1.0"?>
<gazebo:world 
  xmlns:xi="http://www.w3.org/2001/XInclude"
  xmlns:gazebo="http://playerstage.sourceforge.net/gazebo/xmlschema/#gz" 
  xmlns:model="http://playerstage.sourceforge.net/gazebo/xmlschema/#model" 
  xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor" 
  xmlns:param="http://playerstage.sourceforge.net/gazebo/xmlschema/#param" 
  xmlns:body="http://playerstage.sourceforge.net/gazebo/xmlschema/#body" 
  xmlns:geom="http://playerstage.sourceforge.net/gazebo/xmlschema/#geom" 
  xmlns:joint="http://playerstage.sourceforge.net/gazebo/xmlschema/#joint" 
  xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface" 
  xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
  xmlns:physics="http://playerstage.sourceforge.net/gazebo/xmlschema/#physics"
  xmlns:visual="http://www.ieeta.pt/atri/cambada/xmlschema/#visual"
  xmlns:referee="http://www.ieeta.pt/atri/cambada/xmlschema/#referee"
  xmlns:field="http://www.ieeta.pt/atri/cambada/xmlschema/#field" >

  <verbosity><%= simconf['verbosity'] %></verbosity>
  <logData><%= simconf['logData'] %></logData>
  <logJson><%= simconf['logJson'] %></logJson>

  <physics:ode>
    <stepTime><%= simconf['physics']['steptime'] %></stepTime>
    <gravity>0 0 <%= simconf['physics']['gravity'] %></gravity>
    <cfm>10e-5</cfm>
    <erp>0.8</erp>
    <updateRate>-1</updateRate>
  </physics:ode>
  
  <visual:qt4>
    <ballModel><%= simconf['ball_of_game']['name'] %></ballModel>
  </visual:qt4>

  <field:msl>
    <fieldLength><%= simconf['field']['length'] %></fieldLength>
    <fieldWidth><%= simconf['field']['width'] %></fieldWidth>
    
    <centerCircleRadius><%= simconf['field']['center_radius'] %></centerCircleRadius>
    <goalieAreaLength><%= simconf['field']['goalie_length'] %></goalieAreaLength>
    <goalieAreaWidth><%= simconf['field']['goalie_width'] %></goalieAreaWidth>

    <penaltyAreaLength><%= simconf['field']['penalty_length'] %></penaltyAreaLength>
    <penaltyAreaWidth><%= simconf['field']['penalty_width'] %></penaltyAreaWidth>
    
    <north><%= simconf['field']['theNorth'] %></north>
  </field:msl>

  <referee:msl>
    <ballModel><%= simconf['ball_of_game']['name'] %></ballModel>
    <contactSensor>ball_contact</contactSensor>
  </referee:msl>

  <model:empty name="CAMBADA_comm">
    <static>true</static>
    
    <controller:comm name="comm">
      <updateRate>10</updateRate>
      <withIFace>false</withIFace>
    </controller:comm>
  </model:empty>

  <model:physical name="plane1_model">
    <xyz>0 0 0</xyz>
    <rpy>0 0 0</rpy>
    <static>true</static>
    <body:plane name="plane1_body">
      <geom:plane name="plane1_geom">
        <normal>0 0 1</normal>
        <bounce>1</bounce>
        <mu1>10</mu1>
        <slip1>1</slip1>
        <slip2>1</slip2>
      </geom:plane>
    </body:plane>
  </model:physical>
	
  <model:physical name="<%= simconf['ball_of_game']['name'] %>">
    <xyz>0 0 2</xyz>
    <body:sphere name="sphere_body">
      <pigment>1.0 0.454 0.0</pigment>
      <dampingFactor>0.005</dampingFactor>
      <geom:sphere name="sphere_geom">
        <mass>0.04</mass>
        <size>0.12</size>
        <bounce>0.9</bounce>
        <bounceVel>1.0</bounceVel>
				<mu1>3</mu1>
      </geom:sphere>

      <sensor:contact name="ball_contact">
        <geom>sphere_geom</geom>
      </sensor:contact>
		</body:sphere>
  </model:physical>
  
	<model:physical name="south_goal">
    <xyz>0 <%= simconf['field']['length'] / -2.0 %> 0</xyz>
    <rpy>0 0 90</rpy>    
    <include embedded="true">
      <xi:include href="goal_yellow.model" />
    </include>
  </model:physical>
  
  <model:physical name="north_goal">
    <xyz>0 <%= simconf['field']['length'] / 2.0 %> 0</xyz>
    <rpy>0 0 -90</rpy>  
    <include embedded="true">
      <xi:include href="goal_blue.model" />
    </include>
  </model:physical>
  
  <% simconf['agents'].each do |a| %>
  <model:physical name="robbie_<%= a %>">
    <xyz><%= simconf['field']['width'] / 2.0 - 0.5%> <%= simconf['field']['length'] / -2.0  + 3.0 + a.to_i  %> 0.2</xyz>
    <rpy>0 0 90</rpy>
    
    <selfID><%= a %></selfID>
    <team>magenta</team>
    <include embedded="true">
      <xi:include href="cambada.model" />
    </include>
  </model:physical>
  <% end %>
  
    <% simconf['obstacles'].times do |a| %>
  <model:physical name="obstacle_<%= a %>">
    <xyz><%= simconf['field']['width'] / 2.0 + 1 %> <%= simconf['field']['length'] / -2.0  + 3.0 + a.to_i  %> 0.2</xyz>
    <rpy>0 0 90</rpy>

    <include embedded="true">
      <xi:include href="obstacle.model" />
    </include>
  </model:physical>
  <% end %>
  
</gazebo:world>
ENDDATA
